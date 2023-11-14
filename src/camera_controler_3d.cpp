#include "camera_controler_3d.h"

// Godot includes
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/input.hpp>

// Utility includes
#include <godot_cpp/variant/utility_functions.hpp>

// Project includes
#include "game_manager.h"

using namespace godot;

void CameraController3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_paused_state", "is_paused"), &CameraController3D::set_paused_state);

    BIND_GETTER_SETTER(CameraController3D, cam_move_speed, PropertyInfo(Variant::FLOAT, "cam_move_speed", PROPERTY_HINT_RANGE, "0.1,10,0.1"));
    BIND_GETTER_SETTER(CameraController3D, cam_rotation_speed, PropertyInfo(Variant::FLOAT, "cam_rotation_speed", PROPERTY_HINT_RANGE, "0.1,10,0.1"));

    BIND_GETTER_SETTER(CameraController3D, focus_object_path, PropertyInfo(Variant::NODE_PATH, "focus_object_path", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));

    BIND_GETTER_SETTER(CameraController3D, focus_height_offset, PropertyInfo(Variant::FLOAT, "focus_height_offset", PROPERTY_HINT_RANGE, "0.0, 10, 0.1"));
    BIND_GETTER_SETTER(CameraController3D, focus_radius_offset, PropertyInfo(Variant::FLOAT, "focus_radius_offset", PROPERTY_HINT_RANGE, "0.0, 10, 0.1"));
    BIND_GETTER_SETTER(CameraController3D, focus_rotation_offset, PropertyInfo(Variant::FLOAT, "focus_rotation_offset", PROPERTY_HINT_RANGE, "0.001,6.263,0.01"));

    BIND_GETTER_SETTER(CameraController3D, orbit_distance, PropertyInfo(Variant::FLOAT, "orbit_distance", PROPERTY_HINT_RANGE, "0.1,20,0.1"));
    BIND_GETTER_SETTER(CameraController3D, orbit_angle, PropertyInfo(Variant::FLOAT, "orbit_angle", PROPERTY_HINT_RANGE, "0.001,6.263,0.01"));
    BIND_GETTER_SETTER(CameraController3D, orbit_rotation, PropertyInfo(Variant::FLOAT, "orbit_rotation", PROPERTY_HINT_RANGE, "0.001,6.263,0.01"));

    BIND_GETTER_SETTER(CameraController3D, dead_zone_radius, PropertyInfo(Variant::FLOAT, "dead_zone_radius", PROPERTY_HINT_RANGE, "0.1,20,0.1"));
}

bool CameraController3D::is_valid() {
    return is_node_ready() && cam_ != nullptr && focus_object_ != nullptr;
}

void CameraController3D::update_target_transform() {
    if (is_valid()) {
        focus_pos_offset_ = Vector3(
            focus_radius_offset_ * cos(focus_rotation_offset_), 
            focus_height_offset_, 
            focus_radius_offset_ * sin(focus_rotation_offset_)
        );
        
        Vector3 target_lookat_ = focus_object_->get_global_position() + focus_pos_offset_;
        float orbit_radius = orbit_distance_ * cos(orbit_angle_);
        Vector3 cam_pos = Vector3(
            target_lookat_.x + orbit_radius * cos(orbit_rotation_),
            target_lookat_.y + orbit_distance_ * sin(orbit_angle_),
            target_lookat_.z + orbit_radius * sin(orbit_rotation_)
        );
        Vector3 cam_up = (target_lookat_ - cam_pos).cross(Vector3(0, 1, 0)).cross(target_lookat_ - cam_pos);
        target_transform_.set_look_at(cam_pos, target_lookat_, cam_up);
    }
}

CameraController3D::CameraController3D() :
    cam_(nullptr),
    cam_move_speed_(3.0f),
    cam_rotation_speed_(3.0f),
    is_paused_(false),
    is_input_responsive_(true),
    focus_object_path_(""),
    focus_object_(nullptr),
    focus_height_offset_(0.0f),
    focus_radius_offset_(0.0f),
    focus_rotation_offset_(0.0f),
    focus_pos_offset_(Vector3(0.0f, 0.0f, 0.0f)),
    view_ray_(nullptr),
    orbit_distance_(1.0f),
    orbit_angle_((3.14f / 180.0f) * 30.0f), // 30 Degrees
    orbit_rotation_(0.0f),
    dead_zone_radius_(0.0f),
    target_lookat_(Vector3(0.0f, 0.0f, 0.0f))
{}

CameraController3D::~CameraController3D() {}

void CameraController3D::_ready() {
    TypedArray<Node> children = get_children();
    for (int i = 0; i < children.size(); i++) {
        if (Object::cast_to<Camera3D>(children[i]) != nullptr) {
            cam_ = Object::cast_to<Camera3D>(children[i]);
        }
    }
    if (cam_ == nullptr) {
        UtilityFunctions::printerr("Camera Controller: Could not find required Camera child.");
        children = cam_->get_children();
        for (int i = 0; i < children.size(); i++) {
            if (Object::cast_to<RayCast3D>(children[i]) != nullptr) {
                view_ray_ = Object::cast_to<RayCast3D>(children[i]);
            }
        }
        if (view_ray_ == nullptr) {
            UtilityFunctions::printerr("Camera Controller: could not find a RayCast3D child.");
        }
    }
    if (GameManager::get_singleton() != nullptr) {
        GameManager* gm = GameManager::get_singleton();
        gm->connect("game_pause", Callable(this, "set_paused_state"));
    }
    set_focus_object_path(focus_object_path_);
}

void CameraController3D::_process(double delta) {
    if (!is_valid() || is_paused_) {
        return;
    }
    if (view_ray_ != nullptr) {
        view_ray_->force_raycast_update();
        if (view_ray_->get_collider() != nullptr && view_ray_->get_collider() != focus_object_) {
            // When we hit an object, make it invisible
            CollisionObject3D* hit_obj = Object::cast_to<CollisionObject3D>(view_ray_->get_collider());
            if (hit_obj) {
                hit_obj->set_visible(false);
                hidden_objs_.insert(hit_obj);
            }
        } else {
            // When we aren't colliding anymore with anything, unhide objects and clear hidden list.
            for (CollisionObject3D* obj : hidden_objs_) {
                if (obj)
                    obj->set_visible(true);
            }
            hidden_objs_.clear();
        }
    }
    if ((focus_object_->get_global_position() + focus_pos_offset_).distance_to(target_lookat_) >= dead_zone_radius_) {
        update_target_transform();
    }
    if (!cam_->get_global_transform().is_equal_approx(target_transform_)) {
        // We aren't yet at the target transform
        // Lerp and slerp
        Transform3D current = cam_->get_global_transform();
        current.set_origin(current.origin.lerp(target_transform_.origin, delta * cam_move_speed_));
        Quaternion cur_rot(current.basis);
        current.set_basis(cur_rot.slerp(Quaternion(target_transform_.basis), delta * cam_rotation_speed_));
        cam_->set_global_transform(current);
        if (view_ray_ != nullptr) {
            view_ray_->set_target_position(view_ray_->get_global_transform().xform_inv(focus_object_->get_global_position()));
        }
    }

}

void CameraController3D::add_ignore_collision_object(CollisionObject3D* ignore_obj) {
    if (view_ray_ != nullptr && ignore_obj != nullptr) {
        view_ray_->add_exception(ignore_obj);
    }
}

void CameraController3D::remove_ignore_collision_object(CollisionObject3D* ignore_obj) {
    if (view_ray_ != nullptr && ignore_obj != nullptr) {
        view_ray_->remove_exception(ignore_obj);
    }
}

void CameraController3D::set_paused_state(bool is_paused) {
    is_paused_ = is_paused;
}

//////////////////////////////////////////
//          Getters and Setters         //
//////////////////////////////////////////

float CameraController3D::get_cam_move_speed() const {
    return cam_move_speed_;
}
void CameraController3D::set_cam_move_speed(const float speed) {
    cam_move_speed_ = speed;
}

float CameraController3D::get_cam_rotation_speed() const {
    return cam_rotation_speed_;
}
void CameraController3D::set_cam_rotation_speed(const float speed) {
    cam_rotation_speed_ = speed;
}

NodePath CameraController3D::get_focus_object_path() const {
    return focus_object_path_;
}
void CameraController3D::set_focus_object_path(const NodePath path) {
    focus_object_path_ = path;
    if (focus_object_path_.is_empty()) {
        UtilityFunctions::printerr("Camera Controller: focus_object_path is empty.");
    } else {
        focus_object_ = get_node<Node3D>(focus_object_path_);
        if (focus_object_ == nullptr) {
            UtilityFunctions::printerr("Camera Controller: chosen focus_object is not valid.");
        }
        update_target_transform();
    }
}


float CameraController3D::get_focus_height_offset() const {
    return focus_height_offset_;
}
void CameraController3D::set_focus_height_offset(const float height) {
    focus_height_offset_ = height;
    update_target_transform();
}

float CameraController3D::get_focus_radius_offset() const {
    return focus_radius_offset_;
}
void CameraController3D::set_focus_radius_offset(const float radius) {
    focus_radius_offset_ = radius;
    update_target_transform();
}

float CameraController3D::get_focus_rotation_offset() const {
    return focus_rotation_offset_;
}
void CameraController3D::set_focus_rotation_offset(const float radians) {
    focus_rotation_offset_ = radians;
    update_target_transform();
}

float CameraController3D::get_orbit_distance() const {
    return orbit_distance_;
}
void CameraController3D::set_orbit_distance(const float dist) {
    orbit_distance_ = dist;
    update_target_transform();
}

float CameraController3D::get_orbit_angle() const {
    return orbit_angle_;
}
void CameraController3D::set_orbit_angle(const float radians) {
    orbit_angle_ = radians;
    update_target_transform();
}

float CameraController3D::get_orbit_rotation() const {
    return orbit_rotation_;
}
void CameraController3D::set_orbit_rotation(const float radians) {
    orbit_rotation_ = radians;
    update_target_transform();
}

float CameraController3D::get_dead_zone_radius() const {
    return dead_zone_radius_;
}
void CameraController3D::set_dead_zone_radius(const float radius) {
    dead_zone_radius_ = radius;
}