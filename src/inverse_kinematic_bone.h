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
        Vector3 rotation_constraint_init_;
        Vector3 rotation_constraint_min_;
        Vector3 rotation_constraint_max_;
    protected:
        static void _bind_methods();

    public:
        InverseKinematicBone();
        ~InverseKinematicBone();

        void _ready();

        Node3D* get_bone_node();
        Node3D* get_end_bone_node();


        DECLARE_GETTER_SETTER(NodePath, bone_path)
        DECLARE_GETTER_SETTER(NodePath, bone_end_path)
        DECLARE_GETTER_SETTER(float, rotation_x_constraint_init)
        DECLARE_GETTER_SETTER(float, rotation_y_constraint_init)
        DECLARE_GETTER_SETTER(float, rotation_z_constraint_init)
        DECLARE_GETTER_SETTER(float, rotation_x_constraint_min)
        DECLARE_GETTER_SETTER(float, rotation_y_constraint_min)
        DECLARE_GETTER_SETTER(float, rotation_z_constraint_min)        
        DECLARE_GETTER_SETTER(float, rotation_x_constraint_max)
        DECLARE_GETTER_SETTER(float, rotation_y_constraint_max)
        DECLARE_GETTER_SETTER(float, rotation_z_constraint_max)
    };
}

#endif