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
    BIND_GETTER_SETTER(InverseKinematicChain, root_pos_node_path, PropertyInfo(Variant::NODE_PATH, "root_pos_node_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicChain, joint_paths, PropertyInfo(Variant::ARRAY, "ik_joint_paths", PROPERTY_HINT_ARRAY_TYPE, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicChain, constraint_mins, PropertyInfo(Variant::ARRAY, "ik_joint_min_rotations", PROPERTY_HINT_ARRAY_TYPE, "Vector3"));
    BIND_GETTER_SETTER(InverseKinematicChain, constraint_maxs, PropertyInfo(Variant::ARRAY, "ik_joint_max_rotations", PROPERTY_HINT_ARRAY_TYPE, "Vector3"));

    BIND_GETTER_SETTER(InverseKinematicChain, max_iterations, PropertyInfo(Variant::INT, "max_iterations", PROPERTY_HINT_RANGE, "0,1000,1"));
    BIND_GETTER_SETTER(InverseKinematicChain, target_threshold, PropertyInfo(Variant::FLOAT, "target_threshold", PROPERTY_HINT_RANGE, "0.001,2.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicChain, calculation_threshold, PropertyInfo(Variant::FLOAT, "calculation_threshold", PROPERTY_HINT_RANGE, "0.001,2.0,0.001"));
}

Vector3 InverseKinematicChain::project_point_onto_line(Vector3 point, Vector3 line_dir, Vector3 line_pos) {
    // Note: Might not be correct, so check later
    point -= line_pos;
    return (point.dot(line_dir) / line_dir.dot(line_dir) ) * line_dir.normalized() + line_pos;
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

// void InverseKinematicChain::update_bones() {
//     // Transform3D cumulative;
//     // UtilityFunctions::print("Node updating: ", this->get_name());
//     for (int i = 0; i < joints_.size() - 1; i++) {
//         Node3D* bone_node = ik_bones_[i]->get_bone_node();
//         if (bone_node == nullptr) {
//             continue;
//         }
        
//         // if (i == 0) {
//         //     cumulative = bone_node->get_global_transform();
//         // } else {
//         //     cumulative = cumulative * bone_node->get_transform();
//         // }

//         // Vector3 y_basis = cumulative.basis.xform(joints_[i].direction_to(joints_[i + 1])).normalized();
//         Transform3D old_global_trans = bone_node->get_global_transform();
//         Vector3 y_basis = joints_[i].direction_to(joints_[i + 1]).normalized();
//         // Vector3 x_basis_temp = model_hierarchy_transform_.basis.xform(Vector3(1, 0, 0));
//         Vector3 x_basis_temp = old_global_trans.basis.xform(Vector3(1, 0, 0)).normalized();
//         if (y_basis.dot(x_basis_temp) >= 0.99) {
//             x_basis_temp = Vector3(0, 1, 0);
//         }
//         // Vector3 x_basis_temp = Vector3(1, 0, 0);
//         // Vector3 x_basis_temp = old_global_trans.basis.xform(Vector3(1, 0, 0));
//         Vector3 z_basis = y_basis.cross(x_basis_temp).normalized();
//         Vector3 x_basis = y_basis.cross(z_basis).normalized();
//         // UtilityFunctions::print("IK Chain: basis");
//         // UtilityFunctions::print("IK Chain:    x_basis: ", x_basis);
//         // UtilityFunctions::print("IK Chain:    y_basis: ", y_basis);
//         // UtilityFunctions::print("IK Chain:    z_basis: ", z_basis);
//         Vector3 old_scale = old_global_trans.basis.get_scale();
//         old_global_trans.basis = Basis(x_basis, y_basis, z_basis);
//         old_global_trans.basis.scale(old_scale);
//         // old_global_trans.origin = joints_[i];
//         if (i != ik_bones_.size() - 1)
//             UtilityFunctions::print("Previous child transform: ", ik_bones_[i + 1]->get_bone_node()->get_global_transform());
//         bone_node->set_global_transform(old_global_trans);
//         bone_node->force_update_transform();
//         if (i != ik_bones_.size() - 1)
//             UtilityFunctions::print("New child transform: ", ik_bones_[i + 1]->get_bone_node()->get_global_transform());
//     }
// }

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

// void InverseKinematicChain::update_bone_vec_recursive(InverseKinematicBone* current) {
//     ik_bones_.push_back(current);
//     InverseKinematicBone* child = nullptr;
//     TypedArray<Node> children = current->get_children();
//     for (int i = 0; i < children.size(); i++) {
//         if (Object::cast_to<InverseKinematicBone>(children[i])) {
//             child = Object::cast_to<InverseKinematicBone>(children[i]);
//             break;
//         }
//     }
//     if (child != nullptr) {
//         update_bone_vec_recursive(child);
//     }
// }

// Transform3D InverseKinematicChain::update_model_hierarchy_transform_recursive(Node3D* root, Node3D* cur) {
//     UtilityFunctions::print("Recursive call with root, ", root, " and cur ", cur);
//     if (cur == nullptr || cur == Object::cast_to<Node3D>(get_tree()->get_root())) {
//         UtilityFunctions::printerr("IK Chain: Some parent or grandparent of a bone is not a Node3D.");
//         return root->get_transform(); // Will make an invalid transform
//     }
//     if (root == cur) {
//         return root->get_transform();
//     } else {
//         return update_model_hierarchy_transform_recursive(root, Object::cast_to<Node3D>(cur->get_parent())) * cur->get_transform();
//     }
// }

// void InverseKinematicChain::update_model_hierarchy_transform() {
//     if (model_root_ != nullptr && ik_bones_.size() > 0 && ik_bones_[0]->get_bone_node() != nullptr) {
//         Node3D* parent_of_root_bone = Object::cast_to<Node3D>(ik_bones_[0]->get_bone_node()->get_parent());
//         if (parent_of_root_bone == nullptr) {
//             UtilityFunctions::printerr("IK Chain: Parent of root bone is not a Node3D.");
//         } else {
//             model_hierarchy_transform_ = update_model_hierarchy_transform_recursive(model_root_, parent_of_root_bone);
//         }
//         UtilityFunctions::print("Update_model_heirarchy: Model transform: ", model_hierarchy_transform_);
//     } else {
//         UtilityFunctions::printerr("IK Chain: No model root assigned");
//     }
// }

void InverseKinematicChain::set_paused_state(bool is_paused) {
    is_paused_ = is_paused;
}

InverseKinematicChain::InverseKinematicChain() :
    target_pos_node_(nullptr),
    root_pos_node_(nullptr),
    calculation_threshold_(0.001f),
    target_threshold_(0.001f),
    max_iterations_(10),
    is_paused_(false)
{}

InverseKinematicChain::~InverseKinematicChain() {}

void InverseKinematicChain::_ready() {
    if (GameManager::get_singleton() != nullptr) {
        GameManager* gm = GameManager::get_singleton();
        gm->connect("game_pause", Callable(this, "set_paused_state"));
    }
    // set_root_bone_path(root_bone_path_);
    if (Engine::get_singleton() != nullptr && !Engine::get_singleton()->is_editor_hint()) {
        set_target_pos_path(target_pos_path_);
        set_root_pos_node_path(root_pos_node_path_);
        set_joint_paths(ik_joint_paths_);
        update_distances(); // Do this only once at the begining of the scene to keep distances between joints consistent.
        update_joints();
    }
    // set_model_root_path(model_root_path_);
    // update_joints_and_distances();
}

void InverseKinematicChain::_process(double delta) {
    if (joints_.size() <= 1 || 
        target_pos_node_ == nullptr || 
        root_pos_node_ == nullptr ||
        ik_joints_.size() <= 1 || 
        distances_.size() + 1 != joints_.size() || 
        Engine::get_singleton()->is_editor_hint() || 
        is_paused_ || 
        ik_joints_[ik_joints_.size() - 1] == nullptr) {

        // UtilityFunctions::print("Returing early", joints_.size() <= 1, 
        //         target_pos_node_ == nullptr, 
        //         ik_joints_.size() <= 1, 
        //         ik_joints_.size() != joints_.size());
        return;
    }
    if (ik_joints_[ik_joints_.size() - 1]->get_global_position()
            .distance_to(target_pos_node_->get_global_position()) > calculation_threshold_) {
        joints_[0] = root_pos_node_->get_global_position();
        for (int i = 0; i < joints_.size() - 1; i++) {
            float r_i = joints_[i].distance_to(joints_[i + 1]);
            float gam_i  = distances_[i] / r_i;
            joints_[i + 1] = (1.0f - gam_i) * joints_[i] + gam_i * joints_[i + 1];
        }
        perform_ik();
        update_joint_nodes();
        update_joints();
        // update_model_hierarchy_transform();
        // update_joints_and_distances();
    }
}


Node3D* InverseKinematicChain::get_target_pos_node() {
    return target_pos_node_;
}

// NodePath InverseKinematicChain::get_root_bone_path() const {
//     return root_bone_path_;
// }
// void InverseKinematicChain::set_root_bone_path(const NodePath path) {
//     root_bone_path_ = path;
//     InverseKinematicBone* root = get_node<InverseKinematicBone>(root_bone_path_);
//     if (root == nullptr) {
//         UtilityFunctions::printerr("IK Chain: could not find an IK Bone child.");
//     } else {
//         // Recursively build IK chain
//         if (root->get_child_count() == 0) {
//             UtilityFunctions::printerr("IK Chain: could not build chain, root has no child bone.");
//         } else {
//             UtilityFunctions::print("IK Chain: found root bone at path with child. ", path);
//             ik_bones_.clear();
//             update_bone_vec_recursive(root);
//         }
//     }
// }

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
    while (constraint_mins_.size() >= ik_joints_.size() && constraint_mins_.size() > 0)
        constraint_mins_.pop_back();
    while (constraint_maxs_.size() >= ik_joints_.size() && constraint_maxs_.size() > 0)
        constraint_maxs_.pop_back();
    while (constraint_mins_.size() < ik_joints_.size())
        constraint_mins_.push_back(Vector3(0, 0, 0));
    while (constraint_maxs_.size() < ik_joints_.size())
        constraint_maxs_.push_back(Vector3(360, 360, 360));
}

// Constraint mins
Array InverseKinematicChain::get_constraint_mins() const {
    return constraint_mins_;
}
void InverseKinematicChain::set_constraint_mins(const Array constraint) {
    constraint_mins_ = constraint;
}

// Constraint maxs
Array InverseKinematicChain::get_constraint_maxs() const {
    return constraint_maxs_;
}
void InverseKinematicChain::set_constraint_maxs(const Array constraint) {
    constraint_maxs_ = constraint;
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
    return root_pos_node_path_;
}
void InverseKinematicChain::set_root_pos_node_path(const NodePath path) {
    root_pos_node_path_ = path;
    root_pos_node_ = get_node<Node3D>(root_pos_node_path_);
    if (root_pos_node_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not find root pos Node3D at the given path: ", path);
    } else {
        UtilityFunctions::print("Found root pos Node3D node at path: ", path);
    }
}


// NodePath InverseKinematicChain::get_model_root_path() const {
//     return model_root_path_;
// }
// void InverseKinematicChain::set_model_root_path(const NodePath path) {
//     model_root_path_ = path;
//     model_root_ = get_node<Node3D>(model_root_path_);
//     if (model_root_ == nullptr) {
//         UtilityFunctions::printerr("IK Chain: could not find model root Node3D at the given path: ", path);
//     } else {
//         UtilityFunctions::print("IK Chain: Found model root at path ", path);
//         update_model_hierarchy_transform();
//     }
// }

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