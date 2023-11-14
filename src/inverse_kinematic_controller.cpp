#include "inverse_kinematic_controller.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void InverseKinematicController::_bind_methods() {
    BIND_GETTER_SETTER(InverseKinematicController, collision_rays, PropertyInfo(Variant::ARRAY, "collision_ray_paths", PROPERTY_HINT_ARRAY_TYPE, "NodePath"));
    BIND_GETTER_SETTER(InverseKinematicController, bone_roots, PropertyInfo(Variant::ARRAY, "bone_root_paths", PROPERTY_HINT_ARRAY_TYPE, "NodePath"));
    BIND_GETTER_SETTER(InverseKinematicController, bone_constraints, PropertyInfo(Variant::ARRAY, "bone_constraints", PROPERTY_HINT_ARRAY_TYPE, "String"));
}

InverseKinematicController::InverseKinematicController() 
{}

InverseKinematicController::~InverseKinematicController() {}

void InverseKinematicController::_ready() {
    set_collision_rays(collision_rays_);
    set_bone_roots(bone_roots_);
    set_bone_constraints(bone_constraints_);
}

void InverseKinematicController::_process(double delta) {
    // In process, check our list of bone roots and see
    // if the end-effector of the bone chain is not close to
    // the target position for that bone chain.
    // (When we check these bone roots, we update our internal
    //  bone chain's transforms only if we need to perform inverse kinematics)
    // If it isn't, perform FABRIK on that bone chain,
    // then update the actual bone roots with the calculated bone chain.

    // For now, perform directly on the Node3D
}


Array InverseKinematicController::get_collision_rays() const {
    return collision_rays_;
}
void InverseKinematicController::set_collision_rays(const Array list) {
    collision_rays_ = list;
}

Array InverseKinematicController::get_bone_roots() const {
    return bone_roots_;
}
void InverseKinematicController::set_bone_roots(const Array list) {
    bone_roots_ = list;
    // TODO: Create bone chains from bone roots
    // Bone:
    //      Transform3D - for position and orientation
    //      Endpoint / distance - for knowing how long the bone is.
    //      Constraints - the constraints on a bone
}

Array InverseKinematicController::get_bone_constraints() const {
    return bone_constraints_;
}
void InverseKinematicController::set_bone_constraints(const Array list) {
    bone_constraints_ = list;
    // TODO: Update the constraints for each bone in our current list
}
