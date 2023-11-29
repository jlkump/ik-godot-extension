#ifndef PLAYER_CONTROLLER_H
#define PLAYER_CONTROLLER_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/character_body3d.hpp>

#include "camera_controller_3d.h"
#include "helpers.h"

namespace godot {
    class PlayerController3D : public CharacterBody3D {
        GDCLASS(PlayerController3D, CharacterBody3D)
    private:

        CameraController3D* camera_controller_;
        NodePath camera_controller_path_;
        enum State {
            IDLE,
            MOVE,
            JUMP,
        };
        State player_state_;
        Basis movement_basis_;
        bool is_running_;
        float movement_speed_;
        float running_speed_;
        float rotate_speed_;

        bool is_paused_;

        bool is_valid(); // Whether or not the player controller is valid
        bool is_valid_state(State s); // Whether the desired state is possible
        void set_player_state(State s);
        void on_camera_transform_updated(Transform3D transform);

    protected:
        static void _bind_methods();

    public:
        PlayerController3D();
        ~PlayerController3D();

        void _ready();
        void _process(double delta);

        void set_movement_basis(const Basis& movement_basis);

        void set_paused_state(bool is_paused);

        DECLARE_GETTER_SETTER(NodePath, camera_controller_path)
        DECLARE_GETTER_SETTER(float, move_speed)
        DECLARE_GETTER_SETTER(float, run_speed)
        DECLARE_GETTER_SETTER(float, rotate_speed)
    };
}

#endif