#include "inverse_kinematic_chain.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/window.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

#include "inverse_kinematic_bone.h"
#include "game_manager.h"

using namespace godot;

void InverseKinematicChain::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_paused_state", "is_paused"), &InverseKinematicChain::set_paused_state);

    BIND_GETTER_SETTER(InverseKinematicChain, root_bone_path, PropertyInfo(Variant::NODE_PATH, "root_bone_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "InverseKinematicBone"));
    BIND_GETTER_SETTER(InverseKinematicChain, target_pos_path, PropertyInfo(Variant::NODE_PATH, "target_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicChain, model_root_path, PropertyInfo(Variant::NODE_PATH, "model_root_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));

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
    if (joints_.size() <= 1) {
        // Must have at least 2 bones
        return;
    }
    Vector3 target = model_hierarchy_transform_.affine_inverse().xform(target_pos_->get_global_position());
    UtilityFunctions::print("Target pos is ", target);
    // distances_.size() == joints.size() - 1
    float reach = 0;
    for (int i = 0; i < distances_.size(); i++) {
        reach += distances_[i];
    }
    UtilityFunctions::print("Reach is ", reach, ". Distance is ", joints_[0].distance_to(target));
    if (joints_[0].distance_to(target) > reach) {
        // We can't reach the target
        for (int i = 0; i < joints_.size() - 1; i++) {
            float r_i = joints_[i].distance_to(joints_[i + 1]);
            float gam_i = distances_[i] / r_i;
            joints_[i] = (1.0f - gam_i) * joints_[i + 1] + gam_i * joints_[i];
        }
    } else {
        // We can reach, so perform FABRIK
        Vector3 initial = joints_[0];
        float dif_a = joints_[joints_.size() - 1].distance_to(target);
        int iteration = 0;
        while (dif_a > target_threshold_ && iteration < max_iterations_) {
            // Forward reaching
            joints_[joints_.size() - 1] = target; 
            // Move to target
            // For first bone, don't worry about rotational constraints
            // For subsequent bones, construct a line from 

            Vector3 back_line_dir_;
            Vector3 back_line_pos_;
            for (int i = joints_.size() - 2; i >= 0; i--) {
                // Ignore the first joint in the back track for constraints
                if (i != joints_.size() - 2) {
                    // Perform joint constraint in the range
                }

                // Limit bone length to the right range
                float r_i = joints_[i].distance_to(joints_[i + 1]);
                float gam_i = distances_[i] / r_i;
                joints_[i] = (1.0f - gam_i) * joints_[i + 1] + gam_i * joints_[i];
                // Record the resultant back_line for the next iteration.
                back_line_pos_ = joints_[i];
                back_line_dir_ = joints_[i] - joints_[i + 1];
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

void InverseKinematicChain::update_bones() {
    Transform3D cumulative = Transform3D();
    for (int i = 0; i < joints_.size() - 1; i++) {
        Node3D* bone_node = ik_bones_[i]->get_bone_node();
        if (bone_node == nullptr) {
            continue;
        }
        
        Transform3D old_global_trans = bone_node->get_global_transform();
        // Transform3D old_global_trans = bone_node->get_global_transform();
        // Transform3D old_local_trans = bone_node->get_transform();
        Vector3 y_basis = model_hierarchy_transform_.basis.xform(joints_[i].direction_to(joints_[i + 1])).normalized();
        // Vector3 y_basis = model_hierarchy_transform_.basis.xform(joints_[i].direction_to(joints_[i + 1]).normalized());
        // UtilityFunctions::print("Dir for joint ", i, " is ", y_basis);
        Vector3 z_basis = y_basis.cross(old_global_trans.basis.xform_inv(Vector3(1, 0, 0))).normalized();
        Vector3 x_basis = y_basis.cross(z_basis).normalized();

        // The following doesn't seem to do much, but the goal is to have
        // no spin on the bones
        if (z_basis.dot(old_global_trans.xform(Vector3(0, 0, 1))) < 0.0f) {
            z_basis = -z_basis;
        }
        if (x_basis.dot(old_global_trans.xform(Vector3(1, 0, 0))) < 0.0f) {
            x_basis = -x_basis;
        }
        // UtilityFunctions::print("Final basis vecs: ");
        // UtilityFunctions::print("    x_basis: ", x_basis);
        // UtilityFunctions::print("    y_basis: ", y_basis);
        // UtilityFunctions::print("    z_basis: ", z_basis);
        // old_global_trans.origin = model_hierarchy_transform_.xform(joints_[i]);
        Vector3 old_scale = model_hierarchy_transform_.basis.get_scale();
        // UtilityFunctions::print("Old scale is: ", old_scale);
        // old_global_trans.basis.scale(old_scale);
        old_global_trans.basis = Basis(x_basis, y_basis, z_basis);
        old_global_trans.basis.scale(old_scale);
        old_global_trans.origin = model_hierarchy_transform_.xform(joints_[i]);
        // UtilityFunctions::print("New global pos is: ", old_global_trans.origin, " from old joint pos: ", joints_[i]);
        bone_node->set_global_transform(old_global_trans);
        cumulative = cumulative * bone_node->get_transform();
    }
}

void InverseKinematicChain::update_bone_vec_recursive(InverseKinematicBone* current) {
    ik_bones_.push_back(current);
    InverseKinematicBone* child = nullptr;
    TypedArray<Node> children = current->get_children();
    for (int i = 0; i < children.size(); i++) {
        if (Object::cast_to<InverseKinematicBone>(children[i])) {
            child = Object::cast_to<InverseKinematicBone>(children[i]);
            break;
        }
    }
    if (child != nullptr) {
        update_bone_vec_recursive(child);
    }
}

void InverseKinematicChain::update_joints_and_distances() {
    if (model_root_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: model hierarchy null.");
        return;
    }
    joints_.clear();
    distances_.clear();
    Transform3D model_inv = model_hierarchy_transform_.affine_inverse();
    UtilityFunctions::print("Inverse model is ", model_inv);
    for (int i = 0; i < ik_bones_.size(); i++) {
        if (ik_bones_[i]->get_bone_node() != nullptr && ik_bones_[i]->get_end_bone_node() != nullptr) {
            Vector3 start =  model_inv.xform(ik_bones_[i]->get_bone_node()->get_global_position());
            Vector3 end = model_inv.xform(ik_bones_[i]->get_end_bone_node()->get_global_position());
            joints_.push_back(start);
            distances_.push_back(start.distance_to(end));
            // joints_.push_back(
            //     ik_bones_[i]->get_bone_node()->get_global_position()
            // );
            // distances_.push_back(
            //     ik_bones_[i]->get_bone_node()->get_global_position()
            //     .distance_to(ik_bones_[i]->get_end_bone_node()->get_global_position()));
            UtilityFunctions::print("Updating bone ", i, " with new position ", joints_[i], " and distance ", distances_[i]);
            UtilityFunctions::print("Current model hierarchy transform: ", model_hierarchy_transform_);
        }
    }
    if (ik_bones_[ik_bones_.size() - 1]->get_end_bone_node() != nullptr) {
        Vector3 end = model_inv.xform(ik_bones_[ik_bones_.size() - 1]->get_end_bone_node()->get_global_position());
        joints_.push_back(end);
        UtilityFunctions::print("Updating last bone ", ik_bones_.size(), " with new position ", joints_[ik_bones_.size()]);
    }
    UtilityFunctions::print("Final joints are:");
    for (int i = 0; i < joints_.size(); i++) {
        UtilityFunctions::print("   Joint ", i, ": ", joints_[i]);
    }
}

Transform3D InverseKinematicChain::update_model_hierarchy_transform_recursive(Node3D* root, Node3D* cur) {
    UtilityFunctions::print("Recursive call with root, ", root, " and cur ", cur);
    if (cur == nullptr || cur == Object::cast_to<Node3D>(get_tree()->get_root())) {
        UtilityFunctions::printerr("IK Chain: Some parent or grandparent of a bone is not a Node3D.");
        return root->get_transform(); // Will make an invalid transform
    }
    if (root == cur) {
        return root->get_transform();
    } else {
        return update_model_hierarchy_transform_recursive(root, Object::cast_to<Node3D>(cur->get_parent())) *  cur->get_transform();
    }
}

void InverseKinematicChain::update_model_hierarchy_transform() {
    if (model_root_ != nullptr && ik_bones_.size() > 0 && ik_bones_[0]->get_bone_node() != nullptr) {
        Node3D* parent_of_root_bone = Object::cast_to<Node3D>(ik_bones_[0]->get_bone_node()->get_parent());
        if (parent_of_root_bone == nullptr) {
            UtilityFunctions::printerr("IK Chain: Parent of root bone is not a Node3D.");
        } else {
            model_hierarchy_transform_ = update_model_hierarchy_transform_recursive(model_root_, parent_of_root_bone);
        }
        UtilityFunctions::print("Update_model_heirarchy: Model transform: ", model_hierarchy_transform_);
    } else {
        UtilityFunctions::printerr("IK Chain: No model root assigned");
    }
}

void InverseKinematicChain::set_paused_state(bool is_paused) {
    is_paused_ = is_paused;
}

InverseKinematicChain::InverseKinematicChain() :
    target_pos_(nullptr),
    model_root_(nullptr),
    calculation_threshold_(0.1f),
    target_threshold_(0.01f),
    max_iterations_(10),
    is_paused_(false)
{}

InverseKinematicChain::~InverseKinematicChain() {}

void InverseKinematicChain::_ready() {
    if (GameManager::get_singleton() != nullptr) {
        GameManager* gm = GameManager::get_singleton();
        gm->connect("game_pause", Callable(this, "set_paused_state"));
    }
    set_root_bone_path(root_bone_path_);
    set_target_pos_path(target_pos_path_);
    set_model_root_path(model_root_path_);
    update_joints_and_distances();
}

void InverseKinematicChain::_process(double delta) {
    if (joints_.size() <= 1 || target_pos_ == nullptr || 
            ik_bones_.size() <= 1 || ik_bones_.size() + 1 != joints_.size() || 
            Engine::get_singleton()->is_editor_hint() || is_paused_ 
            || ik_bones_[ik_bones_.size() - 1]->get_end_bone_node() == nullptr) {
        // UtilityFunctions::print("Returing early", joints_.size() <= 1, target_pos_ == nullptr, ik_bones_.size() <= 1, ik_bones_.size() != joints_.size());
        return;
    }
    if (ik_bones_[ik_bones_.size() - 1]->get_end_bone_node()->get_global_position().distance_to(target_pos_->get_global_position()) > calculation_threshold_) {
        // joints_[0] = model_hierarchy_transform_.xform(ik_bones_[ik_bones_.size() - 1]->get_bone_node()->get_global_position());
        perform_ik();
        update_bones();
        update_model_hierarchy_transform();
    }
}


Node3D* InverseKinematicChain::get_target_pos_node() {
    return target_pos_;
}

NodePath InverseKinematicChain::get_root_bone_path() const {
    return root_bone_path_;
}
void InverseKinematicChain::set_root_bone_path(const NodePath path) {
    root_bone_path_ = path;
    InverseKinematicBone* root = get_node<InverseKinematicBone>(root_bone_path_);
    if (root == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not find an IK Bone child.");
    } else {
        // Recursively build IK chain
        if (root->get_child_count() == 0) {
            UtilityFunctions::printerr("IK Chain: could not build chain, root has no child bone.");
        } else {
            UtilityFunctions::print("IK Chain: found root bone at path with child. ", path);
            ik_bones_.clear();
            update_bone_vec_recursive(root);
        }
    }
}

NodePath InverseKinematicChain::get_target_pos_path() const {
    return target_pos_path_;
}
void InverseKinematicChain::set_target_pos_path(const NodePath path) {
    target_pos_path_ = path;
    target_pos_ = get_node<Node3D>(target_pos_path_);
    if (target_pos_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not find target Node3D at the given path: ", path);
    }
}

NodePath InverseKinematicChain::get_model_root_path() const {
    return model_root_path_;
}
void InverseKinematicChain::set_model_root_path(const NodePath path) {
    model_root_path_ = path;
    model_root_ = get_node<Node3D>(model_root_path_);
    if (model_root_ == nullptr) {
        UtilityFunctions::printerr("IK Chain: could not find model root Node3D at the given path: ", path);
    } else {
        UtilityFunctions::print("IK Chain: Found model root at path ", path);
        update_model_hierarchy_transform();
    }
}

float InverseKinematicChain::get_target_threshold() const {
    return target_threshold_;
}
void InverseKinematicChain::set_target_threshold(const float threshold) {
    target_threshold_ = threshold;
}

float InverseKinematicChain::get_calculation_threshold() const {
    return calculation_threshold_;
}
void InverseKinematicChain::set_calculation_threshold(const float threshold) {
    calculation_threshold_ = threshold;
}

int InverseKinematicChain::get_max_iterations() const {
    return max_iterations_;
}
void InverseKinematicChain::set_max_iterations(const int max) {
    max_iterations_ = max;
}