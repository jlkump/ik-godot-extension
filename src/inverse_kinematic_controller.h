#ifndef INVERSE_KINEMATIC_CONTROLLER_H
#define INVERSE_KINEMATIC_CONTROLLER_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>

#include <vector>

#include "helpers.h"
#include "inverse_kinematic_chain.h"

namespace godot {
    class InverseKinematicController : public Node {
        GDCLASS(InverseKinematicController, Node)
    private:
        Array ik_chain_paths_;
        std::vector<InverseKinematicChain*> ik_chains_;

        Array ik_ray_paths_;
        std::vector<RayCast3D*> ik_rays_;
        
        float ik_update_distance_;
        float walking_speed_;

        std::vector<Vector3> starting_pos_;
        std::vector<Vector3> desired_pos_;
        std::vector<float> parametric_deltas_;

    protected:
        static void _bind_methods();

    public:
        InverseKinematicController();
        ~InverseKinematicController();

        void _ready();
        void _process(double delta);
        
        DECLARE_GETTER_SETTER(Array, ik_chain_paths)
        DECLARE_GETTER_SETTER(Array, ik_ray_paths)
        DECLARE_GETTER_SETTER(float, ik_update_dist)
        DECLARE_GETTER_SETTER(float, ik_walking_speed)
    };
}

#endif