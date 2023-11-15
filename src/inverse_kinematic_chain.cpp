#include "inverse_kinematic_chain.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

#include "inverse_kinematic_bone.h"

using namespace godot;

void InverseKinematicChain::_bind_methods() {
    BIND_GETTER_SETTER(InverseKinematicChain, root_bone_path, PropertyInfo(Variant::NODE_PATH, "root_bone_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "InverseKinematicBone"));
    BIND_GETTER_SETTER(InverseKinematicChain, target_pos_path, PropertyInfo(Variant::NODE_PATH, "target", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
}

void InverseKinematicChain::perform_ik() {
    if (joints_.size() <= 1) {
        // Must have at least 2 bones
        return;
    }
    Vector3 target = target_pos_->get_global_position();

    // distances_.size() == joints.size() - 1
    float reach = 0;
    for (int i = 0; i < distances_.size(); i++) {
        reach += distances_[i];
    }

    if (joints_[0].distance_to(target) < reach) {
        // We can't reach the target
        for (int i = 0; i < joints_.size() - 1; i++) {
            float r_i = joints_[i].distance_to(joints_[i + 1]);
            float gam_i = distances_[i] / r_i;
            joints_[i] = (1.0f - gam_i) * joints_[i + 1] + gam_i * joints_[i];
        }
    } else {
        // We can reach, so perform FABRIK
        Vector3 initial = joints_[0];
        float dif_a = initial.distance_to(target);
        int iteration = 0;
        while (dif_a > target_threshold_ || iteration > max_iterations_) {
            joints_[joints_.size() - 1] = target;
            for (int i = joints_.size() - 1; i >= 0; i--) {
                float r_i = joints_[i].distance_to(joints_[i + 1]);
                float gam_i = distances_[i] / r_i;
                joints_[i] = (1.0f - gam_i) * joints_[i + 1] + gam_i * joints_[i];
            }
            for (int i = 0; i < joints_.size() - 1; i++) {
                float r_i = joints_[i].distance_to(joints_[i + 1]);
                float gam_i  = distances_[i] / r_i;
                joints_[i] = (1.0f - gam_i) * joints_[i + 1] + gam_i * joints_[i];
            }
            dif_a = joints_[0].distance_to(target);
            iteration++;
        }
    }
}

void InverseKinematicChain::update_bones() {
    for (int i = 0; i < joints_.size() - 1; i++) {
        Node3D* bone_node = ik_bones_[i]->get_bone_node();
        if (bone_node == nullptr) {
            continue;
        }
        Vector3 look = joints_[i].direction_to(joints_[i + 1]);
        Vector3 prev_up = bone_node->get_global_transform().xform(Vector3(0, 1, 0));
        Vector3 up = look.cross(prev_up).cross(look);
        Transform3D new_trans;
        new_trans.set_look_at(joints_[i], joints_[i + 1], up);
        bone_node->set_global_transform(new_trans);
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
    joints_.clear();
    distances_.clear();
    for (int i = 0; i < ik_bones_.size(); i++) {
        if (ik_bones_[i]->get_bone_node() != nullptr && ik_bones_[i]->get_end_bone_node() != nullptr) {
            joints_.push_back(ik_bones_[i]->get_bone_node()->get_global_position());
            distances_.push_back(ik_bones_[i]->get_bone_node()->get_global_position().distance_to(ik_bones_[i]->get_end_bone_node()->get_global_position()));
        }
    }
    if (ik_bones_[ik_bones_.size() - 1]->get_end_bone_node() != nullptr) {
        joints_.push_back(ik_bones_[ik_bones_.size() - 1]->get_end_bone_node()->get_global_position());
    }
}

InverseKinematicChain::InverseKinematicChain() :
    target_pos_(nullptr),
    calculation_threshold_(0.1f),
    target_threshold_(0.01f),
    max_iterations_(20)
{}

InverseKinematicChain::~InverseKinematicChain() {}

void InverseKinematicChain::_ready() {
    set_root_bone_path(root_bone_path_);
    set_target_pos_path(target_pos_path_);
}

void InverseKinematicChain::_process(double delta) {
    if (joints_.size() <= 1 || target_pos_ == nullptr || ik_bones_.size() <= 1 || ik_bones_.size() != joints_.size()) {
        return;
    }
    if (joints_[0].distance_to(target_pos_->get_global_position()) > calculation_threshold_) {
        perform_ik();
        update_bones();
    }
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
            ik_bones_.clear();
            update_bone_vec_recursive(root);
            update_joints_and_distances();
        }
    }
}

Node3D* InverseKinematicChain::get_target_pos_node() {
    return target_pos_;
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