#include "player_controller_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

#include "camera_controller_3d.h"
#include "inverse_kinematic_controller.h"
#include "game_manager.h"

using namespace godot;

void PlayerController3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_paused_state", "is_paused"), &PlayerController3D::set_paused_state);
    ClassDB::bind_method(D_METHOD("on_camera_transform_updated", "transform"), &PlayerController3D::on_camera_transform_updated);

    BIND_GETTER_SETTER(PlayerController3D, camera_controller_path, PropertyInfo(Variant::NODE_PATH, "camera_controller_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "CameraController3D"))
    BIND_GETTER_SETTER(PlayerController3D, falling_ray_path, PropertyInfo(Variant::NODE_PATH, "falling_ray_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "RayCast3D"))
    BIND_GETTER_SETTER(PlayerController3D, leg_collider_paths, PropertyInfo(Variant::ARRAY, "leg_colliders", PROPERTY_HINT_ARRAY_TYPE, "NodePath"))
    BIND_GETTER_SETTER(PlayerController3D, body_height, PropertyInfo(Variant::FLOAT, "body_height", PROPERTY_HINT_RANGE, "0.01,10.0,0.01"));
    BIND_GETTER_SETTER(PlayerController3D, adjustment_speed, PropertyInfo(Variant::FLOAT, "adjustment_speed", PROPERTY_HINT_RANGE, "0.01,10.0,0.01"));
    BIND_GETTER_SETTER(PlayerController3D, adjustment_threshold, PropertyInfo(Variant::FLOAT, "adjustment_threshold", PROPERTY_HINT_RANGE, "0.01,10.0,0.01"));

    BIND_GETTER_SETTER(PlayerController3D, move_speed, PropertyInfo(Variant::FLOAT, "move_speed", PROPERTY_HINT_RANGE, "0.1,200.0,0.01"));
    BIND_GETTER_SETTER(PlayerController3D, run_speed, PropertyInfo(Variant::FLOAT, "run_speed", PROPERTY_HINT_RANGE, "0.1,200.0,0.01"));
    BIND_GETTER_SETTER(PlayerController3D, rotate_speed, PropertyInfo(Variant::FLOAT, "rotate_speed", PROPERTY_HINT_RANGE, "0.01,200.0,0.01"));

    BIND_GETTER_SETTER(PlayerController3D, ik_controller_path, PropertyInfo(Variant::NODE_PATH, "ik_controller_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "InverseKinematicController"))

    ClassDB::bind_method(D_METHOD("get_ik_abs_path"), &PlayerController3D::get_absolute_ik_con_path);
    ClassDB::bind_method(D_METHOD("get_ik_con_obj"), &PlayerController3D::get_ik_con_obj);
}

bool PlayerController3D::is_valid() {
    return is_node_ready();
}

bool PlayerController3D::is_valid_state(State s) {
    switch (s) {
    case JUMP:
        return false;
    default:
        return true;
    }
}

void PlayerController3D::set_player_state(State s) {
    player_state_ = s;
}

void PlayerController3D::on_camera_transform_updated(Transform3D transform) {
    Vector3 y_basis = Vector3(0, 1, 0);
    Vector3 x_basis = transform.basis.xform(Vector3(1, 0, 0));
    Vector3 z_basis = x_basis.cross(y_basis);
    set_movement_basis(Basis(x_basis, y_basis, z_basis));
}

int PlayerController3D::num_legs_colliding() {
    int count = 0;
    for (int i = 0; i < leg_colliders_.size(); i++) {
        leg_colliders_[i]->force_raycast_update();
        if (leg_colliders_[i]->is_colliding()) {
            count++;
        }
    }
    return count;
}

PlayerController3D::PlayerController3D() :
    player_state_(IDLE),
    is_paused_(false),
    movement_speed_(1.0f),
    running_speed_(1.6f),
    rotate_speed_(0.05f),
    camera_controller_(nullptr),
    falling_ray_(nullptr),
    body_height_(1.0f),
    body_adjustment_speed_(0.5f),
    body_adjustment_theshold_(0.2f)
    // ik_controller_(nullptr)
{}

PlayerController3D::~PlayerController3D() {}

void PlayerController3D::_ready() {
    if (GameManager::get_singleton() != nullptr) {
        GameManager* gm = GameManager::get_singleton();
        gm->connect("game_pause", Callable(this, "set_paused_state"));
    }
    set_player_state(player_state_);
    set_camera_controller_path(camera_controller_path_);
    set_falling_ray_path(falling_ray_path_);
    set_leg_collider_paths(leg_collider_paths_);
}

void PlayerController3D::_process(double delta) {
    if (Engine::get_singleton() == nullptr || Engine::get_singleton()->is_editor_hint() || is_paused_ || !is_valid()) {
        return;
    }

    float rotate_value = 0.0f;
    Vector2 input_vector = Vector2(0.0f, 0.0f);
    if (Input::get_singleton()->is_action_pressed("move_forward")) {
        input_vector.y += 1;
    }
    if (Input::get_singleton()->is_action_pressed("move_back")) {
        input_vector.y -= 1;
    }
    if (Input::get_singleton()->is_action_pressed("move_left")) {
        input_vector.x -= 1;
    }
    if (Input::get_singleton()->is_action_pressed("move_right")) {
        input_vector.x += 1;
    }
    if (Input::get_singleton()->is_action_pressed("rotate_ccw")) {
        rotate_value -= 1.0f;
    }
    if (Input::get_singleton()->is_action_pressed("rotate_cw")) {
        rotate_value += 1.0f;
    }

    if (Input::get_singleton()->is_action_just_pressed("run")) {
        is_running_ = true;
    }
    if (Input::get_singleton()->is_action_just_released("run")) {
        is_running_ = false;
    }
    // Decide the state we will move into
    // By default, if we aren't recieving any input, we enter idle
    if (is_valid_state(IDLE) && input_vector.length() <= 0.001f) {
        set_player_state(IDLE);
    }
    if (is_valid_state(MOVE) && input_vector.length() > 0.001f) {
        set_player_state(MOVE);
    }
    if (is_valid_state(JUMP) && Input::get_singleton()->is_action_just_pressed("jump")) {
        set_player_state(JUMP);
    }

    Vector3 move_vector = Vector3(0, 0, 0);
    switch (player_state_) {
        case IDLE:
            break;
        case MOVE:
            move_vector = (
                -input_vector.y * Vector3(movement_basis_.xform(Vector3(0, 0, 1)).x, 0, movement_basis_.xform(Vector3(0, 0, 1)).z)
                + input_vector.x * Vector3(movement_basis_.xform(Vector3(1, 0, 0)).x, 0, movement_basis_.xform(Vector3(1, 0, 0)).z)
            ).normalized();
            if (is_running_) {
                move_vector *= running_speed_;
            } else {
                move_vector *= movement_speed_;
            }
            break;
        case JUMP:
            if (is_valid_state(JUMP)) {
                move_vector.y += 1.0f;
            }
            break;
    }
    if (player_state_ != JUMP) {
        rotate_object_local(Vector3(0, 1, 0), rotate_value * rotate_speed_);
    }
    if (input_vector.x != 0.0f || input_vector.y != 0.0f) {
        Vector3 y_basis = Vector3(0, 1, 0);
        Transform3D new_trans = get_global_transform();
        Vector3 scale = new_trans.get_basis().get_scale();
        new_trans.set_look_at(get_global_position(), get_global_position() - movement_basis_.xform(Vector3(input_vector.y, 0, input_vector.x)));
        new_trans.scale_basis(scale);
        set_global_transform(new_trans);
    }

    if (falling_ray_ != nullptr) {
        falling_ray_->force_raycast_update();
        if (!falling_ray_->is_colliding() && num_legs_colliding() <= 0) {
            move_vector.y = -9.8;
            // UtilityFunctions::print("falling");
        } else if (move_vector.length() > 0.0f) {
            float target_height = -1000.0f;
            for (int i = 0; i < leg_colliders_.size(); i++) {
                if (leg_colliders_[i]->get_global_position().y > target_height) {
                    target_height = leg_colliders_[i]->get_global_position().y;
                }
            }
            // target_height /= leg_colliders_.size();
            target_height += body_height_;
            
            // UtilityFunctions::print("Target height is: ", target_height, " old height is: ", get_global_position().y);

            if (target_height - get_global_position().y < body_adjustment_theshold_) {
                // UtilityFunctions::print("Moving up");
                move_vector.y -= body_adjustment_speed_;
            } else if (target_height - get_global_position().y > body_adjustment_theshold_) {
                // UtilityFunctions::print("Moving down");
                move_vector.y += body_adjustment_speed_;
            } else {
                // UtilityFunctions::print("Not moving");
            }
        }
    }
    set_velocity(move_vector);
    // if (ik_controller_ != nullptr) {
    //     ik_controller_->set_current_movement_vec(move_vector * delta);
    // }
    move_and_slide();
    if (is_on_floor() && player_state_ == JUMP) {
        set_player_state(IDLE);
    }
}

void PlayerController3D::set_movement_basis(const Basis& movement_basis) {
    movement_basis_ = movement_basis;
}

void PlayerController3D::set_paused_state(bool is_paused) {
    is_paused_ = is_paused;
}

NodePath PlayerController3D::get_camera_controller_path() const {
    return camera_controller_path_;
}
void PlayerController3D::set_camera_controller_path(const NodePath path) {
    camera_controller_path_ = path;
    if (camera_controller_ != nullptr) {
        camera_controller_->disconnect("camera_transform_updated", Callable(this, "on_camera_transform_updated"));
    }
    camera_controller_ = get_node<CameraController3D>(camera_controller_path_);
    if (camera_controller_ == nullptr) {
        UtilityFunctions::printerr("Player Controller: Could not find CameraController3D at given path: ", path);
    } else {
        camera_controller_->connect("camera_transform_updated", Callable(this, "on_camera_transform_updated"));
    }
}

NodePath PlayerController3D::get_falling_ray_path() const {
    return falling_ray_path_;
}
void PlayerController3D::set_falling_ray_path(const NodePath path) {
    falling_ray_path_ = path;
    falling_ray_ = get_node<RayCast3D>(falling_ray_path_);
    if (falling_ray_ == nullptr) {
        UtilityFunctions::printerr("Player Controller: Could not find RayCast3D at given path: ", path);
    }
}

Array PlayerController3D::get_leg_collider_paths() const {
    return leg_collider_paths_;
}
void PlayerController3D::set_leg_collider_paths(const Array paths) {
    leg_collider_paths_ = paths;
    leg_colliders_.clear();
    for (int i = 0; i < leg_collider_paths_.size(); i++) {
        if (get_node<RayCast3D>(leg_collider_paths_[i]) != nullptr) {
            leg_colliders_.push_back(get_node<RayCast3D>(leg_collider_paths_[i]));
        }
    }
}

float PlayerController3D::get_move_speed() const {
    return movement_speed_;
}
void PlayerController3D::set_move_speed(const float speed) {
    movement_speed_ = speed;
}

float PlayerController3D::get_run_speed() const {
    return running_speed_;
}
void PlayerController3D::set_run_speed(const float speed) {
    running_speed_ = speed;
}

float PlayerController3D::get_rotate_speed() const {
    return rotate_speed_;
}
void PlayerController3D::set_rotate_speed(const float speed) {
    rotate_speed_ = speed;
}

float PlayerController3D::get_body_height() const {
    return body_height_;
}
void PlayerController3D::set_body_height(const float height) {
    body_height_ = height;
}

float PlayerController3D::get_adjustment_speed() const {
    return body_adjustment_speed_;
}
void PlayerController3D::set_adjustment_speed(const float speed) {
    body_adjustment_speed_ = speed;
}

float PlayerController3D::get_adjustment_threshold() const {
    return body_adjustment_theshold_;
}
void PlayerController3D::set_adjustment_threshold(const float theshold) {
    body_adjustment_theshold_ = theshold;
}

NodePath PlayerController3D::get_ik_controller_path() const {
    return ik_controller_path_;
}
void PlayerController3D::set_ik_controller_path(const NodePath path) {
    ik_controller_path_ = path;
}


NodePath PlayerController3D::get_absolute_ik_con_path() const {
    if (get_node<InverseKinematicController>(ik_controller_path_))
        return get_node<InverseKinematicController>(ik_controller_path_)->get_path();
    else
        return NodePath();
}

Object* PlayerController3D::get_ik_con_obj() const {
    return get_node<InverseKinematicController>(ik_controller_path_);
}