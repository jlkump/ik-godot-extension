#include "inverse_kinematic_chain.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/window.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

#include "game_manager.h"

using namespace godot;

void InverseKinematicChain::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_paused_state", "is_paused"), &InverseKinematicChain::set_paused_state);

    BIND_GETTER_SETTER(InverseKinematicChain, target_pos_path, PropertyInfo(Variant::NODE_PATH, "target_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicChain, root_pos_node_path, PropertyInfo(Variant::NODE_PATH, "marker_pos_node_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicChain, joint_paths, PropertyInfo(Variant::ARRAY, "ik_joint_paths", PROPERTY_HINT_ARRAY_TYPE, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicChain, constraint_mins_horizontal, PropertyInfo(Variant::ARRAY, "ik_joint_min_rotations_horizontal", PROPERTY_HINT_ARRAY_TYPE, "Float"));
    BIND_GETTER_SETTER(InverseKinematicChain, constraint_mins_vertical, PropertyInfo(Variant::ARRAY, "ik_joint_min_rotations_vertical", PROPERTY_HINT_ARRAY_TYPE, "FLOAT"));
    BIND_GETTER_SETTER(InverseKinematicChain, constraint_maxs_horizontal, PropertyInfo(Variant::ARRAY, "ik_joint_max_rotations_horizontal", PROPERTY_HINT_ARRAY_TYPE, "Float"));
    BIND_GETTER_SETTER(InverseKinematicChain, constraint_maxs_vertical, PropertyInfo(Variant::ARRAY, "ik_joint_max_rotations_vertical", PROPERTY_HINT_ARRAY_TYPE, "FLOAT"));

    BIND_GETTER_SETTER(InverseKinematicChain, max_iterations, PropertyInfo(Variant::INT, "max_iterations", PROPERTY_HINT_RANGE, "0,1000,1"));
    BIND_GETTER_SETTER(InverseKinematicChain, target_threshold, PropertyInfo(Variant::FLOAT, "target_threshold", PROPERTY_HINT_RANGE, "0.001,2.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicChain, calculation_threshold, PropertyInfo(Variant::FLOAT, "calculation_threshold", PROPERTY_HINT_RANGE, "0.001,2.0,0.001"));

    BIND_GETTER_SETTER(InverseKinematicChain, end_effector_collider_path, PropertyInfo(Variant::NODE_PATH, "end_effector_collision_area_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
}

Vector3 InverseKinematicChain::project_point_onto_line(Vector3 point, Vector3 line_dir, Vector3 line_pos) {
    // Note: Might not be correct, so check later
    point -= line_pos;
    return (point.dot(line_dir) / line_dir.dot(line_dir) ) * line_dir.normalized() + line_pos;
}

void InverseKinematicChain::move_starting_joint_to_marker() {
    // Move to marker node's position
    joints_[0] = marker_pos_node_->get_global_position();

    // Update the chain of bones to keep appropriate distances
    for (int i = 0; i < joints_.size() - 1; i++) {
        float r_i = joints_[i].distance_to(joints_[i + 1]);
        float gam_i  = distances_[i] / r_i;
        joints_[i + 1] = (1.0f - gam_i) * joints_[i] + gam_i * joints_[i + 1];
    }
}

void InverseKinematicChain::perform_ik() {
    if (joints_.size() <= 1 || joints_.size() != distances_.size() + 1) {
        // Must have at least 2 bones and n - 1 distances for n joints
        UtilityFunctions::printerr("IK Chain: Wrong number of joints and distances. Check that all joint nodes are valid.");
        return;
    }

    // Get the target pos in world-space
    Vector3 target = target_pos_node_->get_global_position();

    // Calculate how far our chain can reach
    float reach = 0;
    for (int i = 0; i < distances_.size(); i++) {
        reach += distances_[i];
    }

    UtilityFunctions::print("Reach is ", reach, ". Distance is ", joints_[0].distance_to(target));

    if (joints_[0].distance_to(target) > reach) {
        // We can't reach the target, 
        // simply stretch the chain out from where it currently is.
        // in the closest attempt to the target
        for (int i = 0; i < joints_.size() - 1; i++) {
            float r_i = joints_[i].distance_to(target);
            float gam_i = distances_[i] / r_i;
            joints_[i + 1] = (1.0f - gam_i) * joints_[i] + gam_i * target;
        }
    } else {
        UtilityFunctions::print("Performing FABRIK");
        // We can reach, so perform FABRIK
        Vector3 initial = joints_[0];
        float dif_a = joints_[joints_.size() - 1].distance_to(target);
        int iteration = 0;
        while (dif_a > target_threshold_ && iteration < max_iterations_) {
            // Forward reaching
            // Move last bone to target, then loop through each bone
            // and adjust based on the previous pos of the bone
            // and the desired pos.
            joints_[joints_.size() - 1] = target; 

            // Vector3 back_line_dir_;
            // Vector3 back_line_pos_;
            for (int i = joints_.size() - 2; i >= 0; i--) {
                // THE BELOW IS WRONG, since joints.size() - 2 is the joint we want
                // for constraints
                // Ignore the first joint in the back track for constraints
                // if (i != joints_.size() - 2) {
                //     // Perform joint constraint in the range
                // }

                // Limit bone length to the right range
                float r_i = joints_[i].distance_to(joints_[i + 1]);
                float gam_i = distances_[i] / r_i;
                joints_[i] = (1.0f - gam_i) * joints_[i + 1] + gam_i * joints_[i];
                // // Record the resultant back_line for the next iteration.
                // back_line_pos_ = joints_[i];
                // back_line_dir_ = joints_[i] - joints_[i + 1];
            }
            // Backward reaching
            joints_[0] = initial;
            for (int i = 0; i < joints_.size() - 1; i++) {
                float r_i = joints_[i].distance_to(joints_[i + 1]);
                float gam_i  = distances_[i] / r_i;
                joints_[i + 1] = (1.0f - gam_i) * joints_[i] + gam_i * joints_[i + 1];
            }
            dif_a = joints_[joints_.size() - 1].distance_to(target);
            iteration++;
        }
    }
}

void InverseKinematicChain::update_joint_nodes() {
    for (int i = 0; i < ik_joints_.size(); i++) {
        Node3D* joint_node = ik_joints_[i];
        if (joint_node != nullptr && joint_node->is_node_ready()) {
            // Get the old global transform, which will be modified to become the new transform.
            Transform3D old_global_trans = joint_node->get_global_transform();

            // Construct new basis in world-space
            Vector3 y_basis = joints_[i].direction_to(joints_[i + 1]).normalized();
            Vector3 x_basis_temp = Vector3(1, 0, 0);
            if (y_basis.cross(x_basis_temp).length() <= 0.00001f) {
                // Choose a new direction if the cross product isn't working
                x_basis_temp = Vector3(0, 0, 1);
            }
            Vector3 z_basis = y_basis.cross(x_basis_temp).normalized();
            Vector3 x_basis = y_basis.cross(z_basis).normalized();
            
            // Update the basis, making sure to keep the scale of the previous basis
            Vector3 old_scale = old_global_trans.basis.get_scale();
            old_global_trans.basis = Basis(x_basis, y_basis, z_basis);
            old_global_trans.basis.scale(old_scale);

            // Update the global position
            old_global_trans.origin = joints_[i];

            // Update previous transform, force update in world-space.
            // Force update will also update children transforms.
            joint_node->set_global_transform(old_global_trans);
            UtilityFunctions::print("Updating joint: ", joint_node->get_name(), " to have global pos: ", joints_[i]);
            // joint_node->force_update_transform();
        }
    }
}

void InverseKinematicChain::update_joints() {
    // Clear the previous joint positions and update with the
    // positions of the ik_joint nodes
    joints_.clear();
    for (int i = 0; i < ik_joints_.size(); i++) {
        if (ik_joints_[i] != nullptr && ik_joints_[i]->is_node_ready()) {
            Vector3 start =  ik_joints_[i]->get_global_position();
            joints_.push_back(start);
        }
    }
    UtilityFunctions::print("Updated joints:");
    for (int i = 0; i < joints_.size(); i++) {
        UtilityFunctions::print("   Joint ", i, ": ", joints_[i]);
    }
}

void InverseKinematicChain::update_distances() {
    // Clear the previous joint distances and update with the
    // distances between the ik_joint nodes
    distances_.clear();
    for (int i = 0; i < ik_joints_.size() - 1; i++) {
        if (ik_joints_[i] != nullptr && ik_joints_[i]->is_node_ready()) {
            Vector3 start =  ik_joints_[i]->get_global_position();
            Vector3 end = ik_joints_[i + 1]->get_global_position();
            distances_.push_back(start.distance_to(end));
        }
    }
    UtilityFunctions::print("Updated distances:");
    for (int i = 0; i < distances_.size(); i++) {
        UtilityFunctions::print("   Distance ", i, " to ", i + 1, ": ", distances_[i]);
    }
}

void InverseKinematicChain::set_paused_state(bool is_paused) {
    is_paused_ = is_paused;
}

InverseKinematicChain::InverseKinematicChain() :
    target_pos_node_(nullptr),
    marker_pos_node_(nullptr),
    calculation_threshold_(0.001f),
    target_threshold_(0.001f),
    max_iterations_(10),
    is_paused_(false),
    end_effector_collider_ray_(nullptr)
{}

InverseKinematicChain::~InverseKinematicChain() {}

void InverseKinematicChain::_ready() {
    if (GameManager::get_singleton() != nullptr) {
        GameManager* gm = GameManager::get_singleton();
        gm->connect("game_pause", Callable(this, "set_paused_state"));
    }

    if (Engine::get_singleton() != nullptr && !Engine::get_singleton()->is_editor_hint()) {
        set_target_pos_path(target_pos_path_);
        set_root_pos_node_path(marker_pos_node_path_);
        set_end_effector_collider_path(end_effector_collider_path_);
        set_joint_paths(ik_joint_paths_);
        update_distances(); // Do this only once at the begining of the scene to keep distances between joints consistent.
        update_joints();
    }
}

void InverseKinematicChain::_process(double delta) {
    if (joints_.size() <= 1 || target_pos_node_ == nullptr || marker_pos_node_ == nullptr ||
        ik_joints_.size() <= 1 || distances_.size() + 1 != joints_.size() || 
        Engine::get_singleton()->is_editor_hint() || is_paused_ || 
        ik_joints_[ik_joints_.size() - 1] == nullptr) {

        // UtilityFunctions::print("Returing early", joints_.size() <= 1, 
        //         target_pos_node_ == nullptr, 
        //         ik_joints_.size() <= 1, 
        //         ik_joints_.size() != joints_.size());
        return;
    }

    // If the end effector bone is beyond some threshold from the target, begin calculation for IK
    if (ik_joints_[ik_joints_.size() - 1]->get_global_position().distance_to(target_pos_node_->get_global_position()) > calculation_threshold_) {
        move_starting_joint_to_marker();
        perform_ik();
        update_joint_nodes();
        update_joints();
        // Update the end_effector collider (if it exists) to be placed at the end of the bone's position.
        if (end_effector_collider_ray_ != nullptr) {
            end_effector_collider_ray_->set_global_position(joints_[joints_.size() - 1]);
        }
    }
}

Node3D* InverseKinematicChain::get_target_pos_node() {
    return target_pos_node_;
}

bool InverseKinematicChain::is_end_effector_colliding() {
    if (end_effector_collider_ray_ == nullptr) {
        return false;
    }
    end_effector_collider_ray_->force_raycast_update();
    return end_effector_collider_ray_->is_colliding();
}

Array InverseKinematicChain::get_joint_paths() const {
    return ik_joint_paths_;
}
void InverseKinematicChain::set_joint_paths(const Array paths) {
    ik_joint_paths_ = paths;
    ik_joints_.clear();
    for (int i = 0; i < ik_joint_paths_.size(); i++) {
        Node3D* joint_node = get_node<Node3D>(ik_joint_paths_[i]);
        if (joint_node != nullptr) {
            ik_joints_.push_back(joint_node);
        } else {
            UtilityFunctions::printerr("Got invalid bone joint for array at index: ", i);
        }
    }
    while (constraint_mins_horizontal_.size() >= ik_joints_.size() && constraint_mins_horizontal_.size() > 0)
        constraint_mins_horizontal_.pop_back();
    while (constraint_mins_vertical_.size() >= ik_joints_.size() && constraint_mins_vertical_.size() > 0)
        constraint_mins_vertical_.pop_back();
    while (constraint_maxs_horizontal_.size() >= ik_joints_.size() && constraint_maxs_horizontal_.size() > 0)
        constraint_maxs_horizontal_.pop_back();
    while (constraint_maxs_vertical_.size() >= ik_joints_.size() && constraint_maxs_vertical_.size() > 0)
        constraint_maxs_vertical_.pop_back();
    
    while (constraint_mins_horizontal_.size() < ik_joints_.size())
        constraint_mins_horizontal_.push_back(-180);
    while (constraint_mins_vertical_.size() < ik_joints_.size())
        constraint_mins_vertical_.push_back(-180);
    while (constraint_maxs_horizontal_.size() < ik_joints_.size())
        constraint_maxs_horizontal_.push_back(180);
    while (constraint_maxs_vertical_.size() < ik_joints_.size())
        constraint_maxs_vertical_.push_back(180);
}

// Constraint mins
Array InverseKinematicChain::get_constraint_mins_horizontal() const {
    return constraint_mins_horizontal_;
}
void InverseKinematicChain::set_constraint_mins_horizontal(const Array constraint) {
    constraint_mins_horizontal_ = constraint;
}
Array InverseKinematicChain::get_constraint_mins_vertical() const {
    return constraint_mins_vertical_;
}
void InverseKinematicChain::set_constraint_mins_vertical(const Array constraint) {
    constraint_mins_vertical_ = constraint;
}

// Constraint maxs
Array InverseKinematicChain::get_constraint_maxs_horizontal() const {
    return constraint_maxs_horizontal_;
}
void InverseKinematicChain::set_constraint_maxs_horizontal(const Array constraint) {
    constraint_maxs_horizontal_ = constraint;
}
Array InverseKinematicChain::get_constraint_maxs_vertical() const {
    return constraint_maxs_vertical_;
}
void InverseKinematicChain::set_constraint_maxs_vertical(const Array constraint) {
    constraint_maxs_vertical_ = constraint;
}

// Target pos node path
NodePath InverseKinematicChain::get_target_pos_path() const {
    return target_pos_path_;
}
void InverseKinematicChain::set_target_pos_path(const NodePath path) {
    target_pos_path_ = path;
    target_pos_node_ = get_node<Node3D>(target_pos_path_);
    if (target_pos_node_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not find target Node3D at the given path: ", path);
    } else {
        UtilityFunctions::print("Found target node at path: ", path);
    }
}

// Root pos node path
NodePath InverseKinematicChain::get_root_pos_node_path() const {
    return marker_pos_node_path_;
}
void InverseKinematicChain::set_root_pos_node_path(const NodePath path) {
    marker_pos_node_path_ = path;
    marker_pos_node_ = get_node<Node3D>(marker_pos_node_path_);
    if (marker_pos_node_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not find root pos Node3D at the given path: ", path);
    } else {
        UtilityFunctions::print("Found root pos Node3D node at path: ", path);
    }
}

// Target Threshold
float InverseKinematicChain::get_target_threshold() const {
    return target_threshold_;
}
void InverseKinematicChain::set_target_threshold(const float threshold) {
    target_threshold_ = threshold;
}

// Calculation threshold
float InverseKinematicChain::get_calculation_threshold() const {
    return calculation_threshold_;
}
void InverseKinematicChain::set_calculation_threshold(const float threshold) {
    calculation_threshold_ = threshold;
}

// Max iterations
int InverseKinematicChain::get_max_iterations() const {
    return max_iterations_;
}
void InverseKinematicChain::set_max_iterations(const int max) {
    max_iterations_ = max;
}

// End effector collision area
NodePath InverseKinematicChain::get_end_effector_collider_path() const {
    return end_effector_collider_path_;
}
void InverseKinematicChain::set_end_effector_collider_path(const NodePath path) {
    end_effector_collider_path_ = path;
    end_effector_collider_ray_ = get_node<RayCast3D>(end_effector_collider_path_);
    if (end_effector_collider_ray_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not end effector RayCast3D at given path: ", path);
    } else {
        UtilityFunctions::print("Found end effector RayCast3D node at path: ", path);
    }
}