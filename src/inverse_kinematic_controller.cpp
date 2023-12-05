#include "inverse_kinematic_controller.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

#include "game_manager.h"

using namespace godot;

void InverseKinematicController::_bind_methods() {

    ClassDB::bind_method(D_METHOD("set_paused_state", "is_paused"), &InverseKinematicController::set_paused_state);

    BIND_GETTER_SETTER(InverseKinematicController, ik_chain_paths, PropertyInfo(Variant::ARRAY, "ik_chain_paths", PROPERTY_HINT_ARRAY_TYPE, "NodePath"));
    BIND_GETTER_SETTER(InverseKinematicController, ik_ray_paths, PropertyInfo(Variant::ARRAY, "ik_ray_paths", PROPERTY_HINT_ARRAY_TYPE, "NodePath"));
    BIND_GETTER_SETTER(InverseKinematicController, interp_speed, PropertyInfo(Variant::FLOAT, "interp_speed", PROPERTY_HINT_RANGE, "0.02,10.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicController, stride, PropertyInfo(Variant::FLOAT, "stride", PROPERTY_HINT_RANGE, "0.01,100.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicController, stride_height, PropertyInfo(Variant::FLOAT, "stride_height", PROPERTY_HINT_RANGE, "0.01,100.0,0.001"));
    BIND_GETTER_SETTER(InverseKinematicController, stride_update_dist, PropertyInfo(Variant::FLOAT, "stride_update_distance", PROPERTY_HINT_RANGE, "0.01,100.0,0.001"));
}

InverseKinematicController::InverseKinematicController() : 
    interp_speed_(1.0f),
    stride_(0.3f),
    stride_update_dist_(0.6f),
    stride_height_(0.3f),
    is_paused_(false)
{}

InverseKinematicController::~InverseKinematicController() {}

void InverseKinematicController::initialize_resting_positions() {
    resting_pos_.clear();
    previous_colisions_.clear();
    initial_pos_.clear();
    target_pos_.clear();
    for (int i = 0; i < ik_rays_.size(); i++) {
        RayCast3D* ray = ik_rays_[i];
        if (ray != nullptr) {
            ray->force_raycast_update();
            Vector3 collision_point = ray->get_collision_point();
            initial_pos_.push_back(collision_point);
            previous_colisions_.push_back(collision_point);
            target_pos_.push_back(collision_point);
            resting_pos_.push_back(ray->get_position());
        }
    }
    parametric_deltas_.resize(ik_rays_.size(), 0.0f);
}

void InverseKinematicController::set_paused_state(bool is_paused) {
    is_paused_ = is_paused;
}

void InverseKinematicController::_ready() {
    if (GameManager::get_singleton() != nullptr) {
        GameManager* gm = GameManager::get_singleton();
        gm->connect("game_pause", Callable(this, "set_paused_state"));
    }
    if (Engine::get_singleton() != nullptr && !Engine::get_singleton()->is_editor_hint()) {
        set_ik_chain_paths(ik_chain_paths_);
        set_ik_ray_paths(ik_ray_paths_);
        initialize_resting_positions();
    }
}

void InverseKinematicController::_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint() || is_paused_) {
        return;
    }
    if (ik_rays_.size() != ik_chains_.size() || 
            parametric_deltas_.size() != ik_rays_.size() || 
            initial_pos_.size() != ik_rays_.size() || 
            target_pos_.size() != ik_rays_.size() || 
            resting_pos_.size() != ik_rays_.size()) {
        UtilityFunctions::printerr("ik rays do not match the number of ik chains");
        return;
    }

    for (int i = 0; i < ik_rays_.size(); i++) {
        RayCast3D* ray = ik_rays_[i];
        if (ray != nullptr && ik_chains_[i]->get_target_pos_node() != nullptr) {
            ray->force_raycast_update();
            Vector3 desired_pos;
            Vector3 collision_pos = ray->get_collision_point();
            if (ray->is_colliding()) {
                desired_pos = (collision_pos - previous_colisions_[i])
                    .clamp(Vector3(-stride_, 0, -stride_), Vector3(stride_, 0, stride_)) 
                    + collision_pos;
                previous_colisions_[i] = collision_pos;
            } else {
                // Todo: Have the legs hang rather than go to resting
                // Right now, goes to last valid position
                desired_pos = initial_pos_[i];
            }

            Vector3 cur_end_pos = ik_chains_[i]->get_target_pos_node()->get_global_position();

            // See if we should move the leg
            if (collision_pos.distance_to(target_pos_[i]) > stride_update_dist_) {
                target_pos_[i] = desired_pos;
                initial_pos_[i] = cur_end_pos; // Need to reset this y pos to the floor
                initial_pos_[i].y = collision_pos.y;
                parametric_deltas_[i] = delta * interp_speed_;
            }

            // Update the actual leg with IK
            if (parametric_deltas_[i] <= 1.0f) {
                Vector3 interp_pos = initial_pos_[i].lerp(target_pos_[i], parametric_deltas_[i]);
                interp_pos.y = initial_pos_[i].y + sin(parametric_deltas_[i] * 3.14) * stride_height_;
                parametric_deltas_[i] += delta * interp_speed_;
                ik_chains_[i]->get_target_pos_node()->set_global_position(interp_pos);
            } else {
                ik_chains_[i]->get_target_pos_node()->set_global_position(target_pos_[i]);
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
    for (int i = 0; i < ik_chain_paths_.size(); i++) {
        ik_chains_.push_back(get_node<InverseKinematicChain>(ik_chain_paths_[i]));
        if (get_node<InverseKinematicChain>(ik_chain_paths_[i]) == nullptr) {
            UtilityFunctions::printerr("IK Controller: Given path at index ", i, " does not point to a IK Chain.");
        }
    }
}

Array InverseKinematicController::get_ik_ray_paths() const {
    return ik_ray_paths_;
}
void InverseKinematicController::set_ik_ray_paths(const Array paths) {
    ik_ray_paths_ = paths;
    ik_rays_.clear();
    for (int i = 0; i < ik_ray_paths_.size(); i++) {
        RayCast3D* ray = get_node<RayCast3D>(ik_ray_paths_[i]);
        ik_rays_.push_back(ray);
        if (ray == nullptr) {
            UtilityFunctions::printerr("IK Controller: Given path at index ", i, " does not point to a RayCast3D.");
        }
    }
}

float InverseKinematicController::get_interp_speed() const {
    return interp_speed_;
}
void InverseKinematicController::set_interp_speed(const float stride) {
    interp_speed_ = stride;
}

float InverseKinematicController::get_stride() const {
    return stride_;
}
void InverseKinematicController::set_stride(const float stride) {
    stride_ = stride;
}

float InverseKinematicController::get_stride_update_dist() const {
    return stride_update_dist_;
}
void InverseKinematicController::set_stride_update_dist(const float dist) {
    stride_update_dist_ = dist;
}

float InverseKinematicController::get_stride_height() const {
    return stride_height_;
}
void InverseKinematicController::set_stride_height(const float height) {
    stride_height_ = height;
}