#include "inverse_kinematic_controller.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void InverseKinematicController::_bind_methods() {
    BIND_GETTER_SETTER(InverseKinematicController, ik_update_dist, PropertyInfo(Variant::FLOAT, "ik_update_dist", PROPERTY_HINT_RANGE, "0.01,5.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicController, ik_walking_speed, PropertyInfo(Variant::FLOAT, "ik_walking_speed", PROPERTY_HINT_RANGE, "0.1,100.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicController, ik_chain_paths, PropertyInfo(Variant::ARRAY, "ik_chain_paths", PROPERTY_HINT_ARRAY_TYPE, "NodePath"));
    BIND_GETTER_SETTER(InverseKinematicController, ik_ray_paths, PropertyInfo(Variant::ARRAY, "ik_ray_paths", PROPERTY_HINT_ARRAY_TYPE, "NodePath"));
}

InverseKinematicController::InverseKinematicController() : 
    ik_update_distance_(0.1f),
    walking_speed_(2.0f)
{

}

InverseKinematicController::~InverseKinematicController() {}

void InverseKinematicController::_ready() {
    set_ik_chain_paths(ik_chain_paths_);
    set_ik_ray_paths(ik_ray_paths_);
}

void InverseKinematicController::_process(double delta) {
    if (ik_rays_.size() != ik_chains_.size() || 
            parametric_deltas_.size() != ik_rays_.size() || 
            desired_pos_.size() != ik_rays_.size() || 
            starting_pos_.size() != ik_rays_.size() || Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    for (int i = 0; i < ik_rays_.size(); i++) {
        RayCast3D* ray = ik_rays_[i];
        if (ray != nullptr && ik_chains_[i]->get_target_pos_node() != nullptr) {
            ray->force_raycast_update();
            // See if we should move the leg
            if (ray->get_collision_point().distance_to(ik_chains_[i]->get_target_pos_node()->get_global_position()) > ik_update_distance_) {
                desired_pos_[i] = ray->get_collision_point();
                starting_pos_[i] = ik_chains_[i]->get_target_pos_node()->get_global_position();
                parametric_deltas_[i] = 0.0f;
            }

            // Update the actual leg with IK
            if (desired_pos_[i].distance_to(ik_chains_[i]->get_target_pos_node()->get_global_position()) > 0.0001f) {
                ik_chains_[i]->get_target_pos_node()->set_global_position(desired_pos_[i]);
                parametric_deltas_[i] += delta;
                Vector3 updated_pos = starting_pos_[i].lerp(desired_pos_[i], parametric_deltas_[i] * walking_speed_);
            }
        }

    }
}


Array InverseKinematicController::get_ik_chain_paths() const {
    return ik_chain_paths_;
}
void InverseKinematicController::set_ik_chain_paths(const Array paths) {
    ik_chain_paths_ = paths;
    ik_chains_.clear();
    starting_pos_.clear();
    for (int i = 0; i < ik_chain_paths_.size(); i++) {
        ik_chains_.push_back(get_node<InverseKinematicChain>(ik_chain_paths_[i]));
        if (get_node<InverseKinematicChain>(ik_chain_paths_[i]) == nullptr) {
            UtilityFunctions::printerr("IK Controller: Given path at index ", i, " does not point to a IK Chain.");
        } else {
            if (ik_chains_[i]->get_target_pos_node() != nullptr) {
                starting_pos_.push_back(ik_chains_[i]->get_target_pos_node()->get_global_position());
            }
        }
    }
}

Array InverseKinematicController::get_ik_ray_paths() const {
    return ik_ray_paths_;
}
void InverseKinematicController::set_ik_ray_paths(const Array paths) {
    ik_ray_paths_ = paths;
    ik_rays_.clear();
    desired_pos_.clear();
    for (int i = 0; i < ik_ray_paths_.size(); i++) {
        RayCast3D* ray = get_node<RayCast3D>(ik_ray_paths_[i]);
        ik_rays_.push_back(ray);
        if (ray == nullptr) {
            UtilityFunctions::printerr("IK Controller: Given path at index ", i, " does not point to a RayCast3D.");
        } else {
            ray->force_raycast_update();
            desired_pos_.push_back(ray->get_collision_point());
        }
    }
    parametric_deltas_.resize(ik_rays_.size(), 0.0f);
}

float InverseKinematicController::get_ik_update_dist() const {
    return ik_update_distance_;
}
void InverseKinematicController::set_ik_update_dist(const float dist) {
    ik_update_distance_ = dist;
}

float InverseKinematicController::get_ik_walking_speed() const {
    return walking_speed_;
}
void InverseKinematicController::set_ik_walking_speed(const float speed) {
    walking_speed_ = speed;
}