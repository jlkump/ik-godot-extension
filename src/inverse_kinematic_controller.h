#ifndef INVERSE_KINEMATIC_CONTROLLER_H
#define INVERSE_KINEMATIC_CONTROLLER_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>

#include <vector>

#include "helpers.h"
#include "inverse_kinematic_chain.h"

namespace godot {
    // TODO: Rename to IKWalkerController
    class InverseKinematicController : public Node {
        GDCLASS(InverseKinematicController, Node)
    private:
        Array ik_chain_paths_;
        std::vector<InverseKinematicChain*> ik_chains_;

        Array ik_ray_paths_;
        std::vector<RayCast3D*> ik_rays_;
        std::vector<Vector3> previous_colisions_;

        // We overshoot the ray's resting pos by some amount
        // depending on how fast we are moving.
        float interp_speed_;
        float stride_;
        float stride_update_dist_;
        float stride_height_;

        // Resting pos is the local pos of each ray
        std::vector<Vector3> resting_pos_;
        // Desired pos is the world-space collision of the ray with the ground
        std::vector<Vector3> initial_pos_;
        std::vector<Vector3> target_pos_;
        std::vector<float> parametric_deltas_;  // How far along in the interpolation we are
        // When this exceeds the max for rest time, 
        // we move the desired position back to the resting pos
        // This only gets increased when the legs haven't moved and is reset to 0 on every move
        std::vector<float> time_not_in_resting_;
        float time_return_to_rest_;

        void initialize_resting_positions();
        // inline float calculate_stride();

        bool is_paused_;
        void set_paused_state(bool is_paused);

    protected:
        static void _bind_methods();

    public:
        InverseKinematicController();
        ~InverseKinematicController();

        void _ready();
        void _process(double delta);

        DECLARE_GETTER_SETTER(Array, ik_chain_paths)
        DECLARE_GETTER_SETTER(Array, ik_ray_paths)
        DECLARE_GETTER_SETTER(float, interp_speed)
        DECLARE_GETTER_SETTER(float, stride)
        DECLARE_GETTER_SETTER(float, stride_update_dist)
        DECLARE_GETTER_SETTER(float, stride_height)
    };
}

#endif