#ifndef CAMERA_CONTROLLER_3D_H
#define CAMERA_CONTROLLER_3D_H

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>
#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/remote_transform3d.hpp>
#include <godot_cpp/classes/input.hpp>

#include <unordered_set>

#include "helpers.h"

namespace godot {
    class CameraController3D : public Node {
        GDCLASS(CameraController3D, Node)

    private:
        Camera3D* cam_;
        float cam_move_speed_;
        float cam_rotation_speed_;
        float input_lateral_rotation_speed_;
        float input_vertical_rotation_speed_;

        bool is_paused_;
        bool is_input_responsive_; 

        NodePath focus_object_path_;
        Node3D* focus_object_;
        float focus_height_offset_;
        float focus_radius_offset_;
        float focus_rotation_offset_;
        Vector3 focus_pos_offset_;

        RayCast3D* view_ray_; // Detect collisions with objects between the camera and focused object
        std::unordered_set<CollisionObject3D*> hidden_objs_; // All the objects that are hidden
        float orbit_distance_;
        float orbit_angle_;
        float orbit_rotation_;

        float dead_zone_radius_; // The amount the focus object has to move before the camera starts moving

        Transform3D target_transform_; // The transform the camera is currently lerping towards
        Vector3 target_lookat_;
        // TODO: Add shake?

        bool is_valid();
        // Whenever the properties of the camera change or when the focused obj moves outside
        // the dead_zone_radius, this method is called.
        void update_target_transform(); 
    protected:
        static void _bind_methods();
    public:
        CameraController3D();
        ~CameraController3D();

        void _ready();
        void _process(double delta);
        void _input(const Ref<InputEvent>& event);


        void add_ignore_collision_object(CollisionObject3D* ignore_obj);
        void remove_ignore_collision_object(CollisionObject3D* ignore_obj);

        void set_paused_state(bool is_paused);

        // void rotate_to_lookat(Vector3 look_forward);

        DECLARE_GETTER_SETTER(float, cam_move_speed);
        DECLARE_GETTER_SETTER(float, cam_rotation_speed);
        DECLARE_GETTER_SETTER(float, input_lateral_rotation_speed);
        DECLARE_GETTER_SETTER(float, input_vertical_rotation_speed);

        DECLARE_GETTER_SETTER(NodePath, focus_object_path);
        DECLARE_GETTER_SETTER(float, focus_height_offset);
        DECLARE_GETTER_SETTER(float, focus_radius_offset);
        DECLARE_GETTER_SETTER(float, focus_rotation_offset);

        DECLARE_GETTER_SETTER(float, orbit_distance);
        DECLARE_GETTER_SETTER(float, orbit_angle);
        DECLARE_GETTER_SETTER(float, orbit_rotation);

        DECLARE_GETTER_SETTER(float, dead_zone_radius);

    };
}

#endif