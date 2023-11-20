#ifndef INVERSE_KINEMATIC_BONE_H
#define INVERSE_KINEMATIC_BONE_H

#include <godot_cpp/classes/area3d.hpp>

#include "helpers.h"

namespace godot {
    class InverseKinematicBone : public Node {
        GDCLASS(InverseKinematicBone, Node)
    private:
        Node3D* bone_; // Stores transform (pos and orientation).
        NodePath bone_path_;
        Node3D* bone_end_;
        NodePath bone_end_path_;
        float rotation_horizontal_max_;
        float rotation_horizontal_min_;
        float rotation_vertical_max_;
        float rotation_vertical_min_;

        Vector3 initial_pos_;
        Quaternion initial_rotation_;
    protected:
        static void _bind_methods();

    public:
        InverseKinematicBone();
        ~InverseKinematicBone();

        void _ready();

        Node3D* get_bone_node();
        Node3D* get_end_bone_node();

        Vector3 get_initial_pos() const;
        Quaternion get_initial_rotation() const;
        
        DECLARE_GETTER_SETTER(NodePath, bone_path)
        DECLARE_GETTER_SETTER(NodePath, bone_end_path)
        DECLARE_GETTER_SETTER(float, rotation_horizontal_min)
        DECLARE_GETTER_SETTER(float, rotation_horizontal_max)
        DECLARE_GETTER_SETTER(float, rotation_vertical_min)
        DECLARE_GETTER_SETTER(float, rotation_vertical_max)
    };
}

#endif