#include "player_controller_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <godot_cpp/variant/utility_functions.hpp>

#include "game_manager.h"

using namespace godot;

void PlayerController3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_paused_state", "is_paused"), &PlayerController3D::set_paused_state);
    ClassDB::bind_method(D_METHOD("on_camera_transform_updated", "transform"), &PlayerController3D::on_camera_transform_updated);

    BIND_GETTER_SETTER(PlayerController3D, camera_controller_path, PropertyInfo(Variant::NODE_PATH, "camera_controller_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "CameraController3D"))
}

bool PlayerController3D::is_valid() {
    return player_ != nullptr;
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
    player_(nullptr),
    player_state_(IDLE),
    is_paused_(false),
    movement_speed_(1.0f),
    running_speed_(1.6f),
    camera_controller_(nullptr),
    camera_controller_path_("")
{}

PlayerController3D::~PlayerController3D() {}

void PlayerController3D::_ready() {
    TypedArray<Node> children = get_children();
    for (int i = 0; i < children.size(); i++) {
        if (Object::cast_to<CharacterBody3D>(children[i]) != nullptr) {
            player_ = Object::cast_to<CharacterBody3D>(children[i]);
        }
    }
    if (player_ == nullptr) {
        UtilityFunctions::printerr("Player Controller: Player controller requires a CharacterBody3D as a child.");
    }
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

    Vector2 input_vector = Vector2(0.0f, 0.0f);
    if (Input::get_singleton()->is_action_pressed("move_forward")) {
        input_vector.y += 1;
    }
    if (Input::get_singleton()->is_action_pressed("move_backward")) {
        input_vector.y -= 1;
    }
    if (Input::get_singleton()->is_action_pressed("move_left")) {
        input_vector.x -= 1;
    }
    if (Input::get_singleton()->is_action_pressed("move_right")) {
        input_vector.x += 1;
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
    if (!player_->is_on_floor()) {
        move_vector.y = -9.8f;
    }
    switch (player_state_) {
        case IDLE:
            break;
        case MOVE:
            move_vector = (input_vector.y * movement_basis_.xform(Vector3(0, 0, 1)) + input_vector.x * movement_basis_.xform(Vector3(1, 0, 0))).normalized();
            if (is_running_) {
                move_vector *= running_speed_;
            } else {
                move_vector *= movement_speed_;
            }
            break;
        case JUMP:
            break;
    }
    player_->set_velocity(move_vector);
    player_->move_and_slide();
    if (player_->is_on_floor() && player_state_ == JUMP) {
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
    if (camera_controller_ != nullptr) {
        camera_controller_->disconnect("camera_transform_updated", Callable(this, "on_camera_transform_updated"));
    }
    camera_controller_ = get_node<CameraController3D>(path);
    if (camera_controller_ == nullptr) {
        UtilityFunctions::printerr("Player Controller: Could not find CameraController3D at given path: ", path);
    } else {
        camera_controller_->connect("camera_transorm_updated", Callable(this, "on_camera_transform_updated"));
    }
}