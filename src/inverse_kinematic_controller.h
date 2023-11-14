#ifndef INVERSE_KINEMATIC_CONTROLLER_H
#define INVERSE_KINEMATIC_CONTROLLER_H

#include <godot_cpp/classes/area3d.hpp>

#include "helpers.h"

namespace godot {
    class InverseKinematicController : public Node {
        GDCLASS(InverseKinematicController, Node)
    private:
        Array collision_rays_;
        Array bone_roots_;
        // Questions:
        //      How to apply constraints on a bone from the editor?
        //          - Use array of strings, where the format is 
        //          "initial_rotation_xyz,min_rotation_xyz,max_rotation_xyz"
        //          for example: "0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,90.0,0.0"
        //          could allow for rotation only in the local-y axis by an 
        //          offset of 90 degrees from the initial rotation. 
        Array bone_constraints_;

        struct Bone {
            Node3D* node_;
            float length_;
            Vector3 rotation_constraint_init_;
            Vector3 rotation_constraint_min_;
            Vector3 rotation_constraint_max_; 
        };
    protected:
        static void _bind_methods();

    public:
        InverseKinematicController();
        ~InverseKinematicController();

        void _ready();
        void _process(double delta);
        

        // Future methods
        // std::vector<Vector3> get_bone_target_positions();
        // void set_bone_target_positions(std::vector<Vector3> targets);

        DECLARE_GETTER_SETTER(Array, collision_rays)
        DECLARE_GETTER_SETTER(Array, bone_roots)
        DECLARE_GETTER_SETTER(Array, bone_constraints)
    };
}

#endif