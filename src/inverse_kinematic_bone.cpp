#include "inverse_kinematic_bone.h"

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void InverseKinematicBone::_bind_methods() {
    BIND_GETTER_SETTER(InverseKinematicBone, bone_path, PropertyInfo(Variant::NODE_PATH, "bone_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicBone, bone_end_path, PropertyInfo(Variant::NODE_PATH, "bone_endpoint_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));

    BIND_GETTER_SETTER(InverseKinematicBone, rotation_horizontal_min, PropertyInfo(Variant::FLOAT, "rotation_horizontal_min", PROPERTY_HINT_RANGE, "-0.001,-3.14,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_horizontal_max, PropertyInfo(Variant::FLOAT, "rotation_horizontal_max", PROPERTY_HINT_RANGE, "0.001,3.14,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_vertical_min, PropertyInfo(Variant::FLOAT, "rotation_vertical_min", PROPERTY_HINT_RANGE, "-0.001,-3.14,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_vertical_max, PropertyInfo(Variant::FLOAT, "rotation_vertical_max", PROPERTY_HINT_RANGE, "0.001,3.14,0.01"));
}

InverseKinematicBone::InverseKinematicBone() : 
    bone_(nullptr),
    bone_path_(),
    bone_end_(nullptr),
    bone_end_path_(),
    rotation_horizontal_min_(-3.14f),
    rotation_horizontal_max_(3.14f),
    rotation_vertical_min_(-3.14f),
    rotation_vertical_max_(3.14f)
{}
InverseKinematicBone::~InverseKinematicBone() {}

void InverseKinematicBone::_ready() {
    set_bone_path(bone_path_);
    set_bone_end_path(bone_end_path_);
    if (bone_ != nullptr) {
        initial_pos_ = bone_->get_position();
        initial_rotation_ = Quaternion(bone_->get_rotation());
        // bone_->set_rotation(initial_rotation_.get_euler());
    }
}

Node3D* InverseKinematicBone::get_bone_node() {
    return bone_;
}

Node3D* InverseKinematicBone::get_end_bone_node() {
    return bone_end_;
}

Vector3 InverseKinematicBone::get_initial_pos() const {
    return initial_pos_;
}

Quaternion InverseKinematicBone::get_initial_rotation() const {
    return initial_rotation_;
}

// Path setting
NodePath InverseKinematicBone::get_bone_path() const {
    return bone_path_;
}
void InverseKinematicBone::set_bone_path(const NodePath path) {
    bone_path_ = path;
    bone_ = get_node<Node3D>(bone_path_);
    if (bone_ == nullptr) {
        UtilityFunctions::printerr("IK Bone: given path to begin is not valid: ", path);
    } else {
        UtilityFunctions::print("Found begin bone at path: ", path);
        initial_pos_ = bone_->get_global_position();
    }
}

NodePath InverseKinematicBone::get_bone_end_path() const {
    return bone_end_path_;
}
void InverseKinematicBone::set_bone_end_path(const NodePath path) {
    bone_end_path_ = path;
    bone_end_ = get_node<Node3D>(bone_end_path_);
    if (bone_end_ == nullptr) {
        UtilityFunctions::printerr("IK Bone: given path to end is not valid: ", path);
    } else {
        UtilityFunctions::print("Found end bone at path: ", path);
    }
}

// Getters / Setters
float InverseKinematicBone::get_rotation_horizontal_min() const {
    return rotation_horizontal_min_;
}
void InverseKinematicBone::set_rotation_horizontal_min(const float val) {
    rotation_horizontal_min_ = val;
}

float InverseKinematicBone::get_rotation_horizontal_max() const {
    return rotation_horizontal_max_;
}
void InverseKinematicBone::set_rotation_horizontal_max(const float val) {
    rotation_horizontal_max_ = val;
}

float InverseKinematicBone::get_rotation_vertical_min() const {
    return rotation_vertical_min_;
}
void InverseKinematicBone::set_rotation_vertical_min(const float val) {
    rotation_vertical_min_ = val;
}

float InverseKinematicBone::get_rotation_vertical_max() const {
    return rotation_vertical_max_;
}
void InverseKinematicBone::set_rotation_vertical_max(const float val) {
    rotation_vertical_max_ = val;
}