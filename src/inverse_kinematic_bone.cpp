#include "inverse_kinematic_bone.h"

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void InverseKinematicBone::_bind_methods() {
    BIND_GETTER_SETTER(InverseKinematicBone, bone_path, PropertyInfo(Variant::NODE_PATH, "bone_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
    BIND_GETTER_SETTER(InverseKinematicBone, bone_end_path, PropertyInfo(Variant::NODE_PATH, "bone_endpoint_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));

    BIND_GETTER_SETTER(InverseKinematicBone, rotation_x_constraint_init, PropertyInfo(Variant::FLOAT, "init_rotation_x", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_y_constraint_init, PropertyInfo(Variant::FLOAT, "init_rotation_y", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_z_constraint_init, PropertyInfo(Variant::FLOAT, "init_rotation_z", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));

    BIND_GETTER_SETTER(InverseKinematicBone, rotation_x_constraint_min, PropertyInfo(Variant::FLOAT, "min_rotation_x", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_x_constraint_max, PropertyInfo(Variant::FLOAT, "max_rotation_x", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_y_constraint_min, PropertyInfo(Variant::FLOAT, "min_rotation_y", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_y_constraint_max, PropertyInfo(Variant::FLOAT, "max_rotation_y", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_z_constraint_min, PropertyInfo(Variant::FLOAT, "min_rotation_z", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
    BIND_GETTER_SETTER(InverseKinematicBone, rotation_z_constraint_max, PropertyInfo(Variant::FLOAT, "max_rotation_z", PROPERTY_HINT_RANGE, "0.001,6.28,0.01"));
}

InverseKinematicBone::InverseKinematicBone() : 
    bone_(nullptr),
    bone_path_(),
    bone_end_(nullptr),
    bone_end_path_(),
    rotation_constraint_init_(Vector3(0,0,0)),
    rotation_constraint_min_(Vector3(0,0,0)),
    rotation_constraint_max_(Vector3(360,360,360))
{}
InverseKinematicBone::~InverseKinematicBone() {}

void InverseKinematicBone::_ready() {
    set_bone_path(bone_path_);
    set_bone_end_path(bone_end_path_);
}

Node3D* InverseKinematicBone::get_bone_node() {
    return bone_;
}

Node3D* InverseKinematicBone::get_end_bone_node() {
    return bone_end_;
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
    }
}

// Init constraint
float InverseKinematicBone::get_rotation_x_constraint_init() const {
    return rotation_constraint_init_.x;
}
void InverseKinematicBone::set_rotation_x_constraint_init(const float val) {
    rotation_constraint_init_.x = val;
}

float InverseKinematicBone::get_rotation_y_constraint_init() const {
    return rotation_constraint_init_.y;
}
void InverseKinematicBone::set_rotation_y_constraint_init(const float val) {
    rotation_constraint_init_.y = val;
}

float InverseKinematicBone::get_rotation_z_constraint_init() const {
    return rotation_constraint_init_.z;
}
void InverseKinematicBone::set_rotation_z_constraint_init(const float val) {
    rotation_constraint_init_.z = val;
}

// Min constraint
float InverseKinematicBone::get_rotation_x_constraint_min() const {
    return rotation_constraint_min_.x;
}
void InverseKinematicBone::set_rotation_x_constraint_min(const float val) {
    rotation_constraint_min_.x = val;
}

float InverseKinematicBone::get_rotation_y_constraint_min() const {
    return rotation_constraint_min_.y;
}
void InverseKinematicBone::set_rotation_y_constraint_min(const float val) {
    rotation_constraint_min_.y = val;
}

float InverseKinematicBone::get_rotation_z_constraint_min() const {
    return rotation_constraint_min_.z;
}
void InverseKinematicBone::set_rotation_z_constraint_min(const float val) {
    rotation_constraint_min_.z = val;
}

// Max constraint
float InverseKinematicBone::get_rotation_x_constraint_max() const {
    return rotation_constraint_max_.x;
}
void InverseKinematicBone::set_rotation_x_constraint_max(const float val) {
    rotation_constraint_max_.x = val;
}

float InverseKinematicBone::get_rotation_y_constraint_max() const {
    return rotation_constraint_max_.y;
}
void InverseKinematicBone::set_rotation_y_constraint_max(const float val) {
    rotation_constraint_max_.y = val;
}

float InverseKinematicBone::get_rotation_z_constraint_max() const {
    return rotation_constraint_max_.z;
}
void InverseKinematicBone::set_rotation_z_constraint_max(const float val) {
    rotation_constraint_max_.z = val;
}