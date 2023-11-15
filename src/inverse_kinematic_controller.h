#ifndef INVERSE_KINEMATIC_CONTROLLER_H
#define INVERSE_KINEMATIC_CONTROLLER_H

#include <godot_cpp/classes/area3d.hpp>

#include <vector>

#include "helpers.h"
#include "inverse_kinematic_chain.h"

namespace godot {
    class InverseKinematicController : public Node {
        GDCLASS(InverseKinematicController, Node)
    private:
        std::vector<InverseKinematicChain*> ik_chains_;
        // Questions:
        //      How to apply constraints on a bone from the editor?
        //          - Use array of strings, where the format is 
        //          "initial_rotation_xyz,min_rotation_xyz,max_rotation_xyz"
        //          for example: "0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,90.0,0.0"
        //          could allow for rotation only in the local-y axis by an 
        //          offset of 90 degrees from the initial rotation. 

    protected:
        static void _bind_methods();

    public:
        InverseKinematicController();
        ~InverseKinematicController();

        void _ready();
        void _process(double delta);
        

        // Future methods
        // std::vector<Node3D*> get_bone_target_positions();
    };
}

#endif