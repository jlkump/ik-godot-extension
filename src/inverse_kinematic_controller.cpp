#include "inverse_kinematic_controller.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void InverseKinematicController::_bind_methods() {
}

InverseKinematicController::InverseKinematicController() 
{}

InverseKinematicController::~InverseKinematicController() {}

void InverseKinematicController::_ready() {
    
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