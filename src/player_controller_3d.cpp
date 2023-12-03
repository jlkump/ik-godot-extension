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
    // BIND_GETTER_SETTER(PlayerController3D, ik_controller_path, PropertyInfo(Variant::NODE_PATH, "ik_controller_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "InverseKinematicController"))
    BIND_GETTER_SETTER(PlayerController3D, move_speed, PropertyInfo(Variant::FLOAT, "move_speed", PROPERTY_HINT_RANGE, "0.1,200.0,0.01"));
    BIND_GETTER_SETTER(PlayerController3D, run_speed, PropertyInfo(Variant::FLOAT, "run_speed", PROPERTY_HINT_RANGE, "0.1,200.0,0.01"));
    BIND_GETTER_SETTER(PlayerController3D, rotate_speed, PropertyInfo(Variant::FLOAT, "rotate_speed", PROPERTY_HINT_RANGE, "0.01,200.0,0.01"));
}

bool PlayerController3D::is_valid() {
    return is_node_ready();
}

bool PlayerController3D::is_valid_state(State s) {
    return true;
}

void PlayerController3D::set_player_state(State s) {
    player_state_ = s;
}

void PlayerController3D::on_camera_transform_updated(Transform3D transform) {
    set_movement_basis(transform.basis);
}

PlayerController3D::PlayerController3D() :
    player_state_(IDLE),
    is_paused_(false),
    movement_speed_(1.0f),
    running_speed_(1.6f),
    rotate_speed_(0.05f),
    camera_controller_(nullptr)
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
    if (!is_on_floor()) {
        move_vector.y = -9.8f;
    }
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
            break;
    }
    if (player_state_ != JUMP) {
        rotate_object_local(Vector3(0, 1, 0), rotate_value * rotate_speed_);
    }
    if (move_vector.x != 0.0f && move_vector.z != 0.0f) {
        Vector3 y_basis = Vector3(0, 1, 0);
        Transform3D new_trans = get_global_transform();
        Vector3 scale = new_trans.get_basis().get_scale();
        new_trans.set_look_at(get_global_position(), get_global_position() - movement_basis_.xform(Vector3(1, 0, 0)));
        new_trans.scale_basis(scale);
        set_global_transform(new_trans);
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

// NodePath PlayerController3D::get_ik_controller_path() const {
//     return ik_controller_path_;
// }
// void PlayerController3D::set_ik_controller_path(const NodePath path) {
//     ik_controller_path_ = path;
//     // For future possible signals
//     // if (ik_controller_ != nullptr) {
//     //     camera_controller_->disconnect("camera_transform_updated", Callable(this, "on_camera_transform_updated"));
//     // }
//     ik_controller_ = get_node<InverseKinematicController>(ik_controller_path_);
//     if (ik_controller_ == nullptr) {
//         UtilityFunctions::printerr("Player Controller: Could not find IKController at given path: ", path);
//     }
//     // Future possible signals
//     //  else {
//     //     camera_controller_->connect("camera_transform_updated", Callable(this, "on_camera_transform_updated"));
//     // }
// }

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