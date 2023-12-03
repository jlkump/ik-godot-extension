#ifndef INVERSE_KINEMATIC_CHAIN_H
#define INVERSE_KINEMATIC_CHAIN_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/ray_cast3d.hpp>

#include "helpers.h"


namespace godot {
    class InverseKinematicChain : public Node {
        GDCLASS(InverseKinematicChain, Node)
    private:

        Array ik_joint_paths_;
        std::vector<Node3D*> ik_joints_;
        std::vector<Vector3> joints_; // Positions
        std::vector<float> distances_;
        Array constraint_mins_horizontal_; // Array of floats, which specify the minimum horizontal rotation in degrees for each bone
        Array constraint_mins_vertical_; // Array of floats, which specify the minimum vertical rotation in degrees for each bone
        Array constraint_maxs_horizontal_; // Same as min, specifies constraint max horizontal for each joint. 
        Array constraint_maxs_vertical_; // Same as min, specifies constraint max vertical for each joint. 

        NodePath marker_pos_node_path_;
        Node3D* marker_pos_node_;

        NodePath target_pos_path_;
        Node3D* target_pos_node_;

        float calculation_threshold_;
        float target_threshold_;
        int max_iterations_;

        NodePath end_effector_collider_path_;
        RayCast3D* end_effector_collider_ray_;
        float end_effector_height_offset_;

        Vector3 project_point_onto_line(Vector3 point, Vector3 line_dir, Vector3 line_pos);
        Vector3 recursive_search_point(Vector3 x_basis, Vector3 z_basis, const float a, const float b, Vector3 target, Vector3 cur, Vector3 O, int iter);
        Vector3 apply_rotational_constraint(Vector3 target, Vector3 prev_joint_end, Vector3 prev_joint_start, 
                                            float min_x, float max_x, float min_y, float max_y,
                                            Basis bone_basis);
        void move_starting_joint_to_marker();
        void perform_ik();
        void update_joint_nodes();
        void update_joints();
        void update_distances();

        bool is_paused_;
        void set_paused_state(bool is_paused);

    protected:
        static void _bind_methods();

    public:
        InverseKinematicChain();
        ~InverseKinematicChain();

        void _ready();
        void _process(double delta);
        
        Node3D* get_target_pos_node();
        bool is_end_effector_colliding() const;
        float get_reach() const;

        DECLARE_GETTER_SETTER(Array, joint_paths)
        DECLARE_GETTER_SETTER(Array, constraint_mins_horizontal)
        DECLARE_GETTER_SETTER(Array, constraint_mins_vertical)
        DECLARE_GETTER_SETTER(Array, constraint_maxs_horizontal)
        DECLARE_GETTER_SETTER(Array, constraint_maxs_vertical)
        DECLARE_GETTER_SETTER(NodePath, target_pos_path)
        DECLARE_GETTER_SETTER(NodePath, root_pos_node_path)

        DECLARE_GETTER_SETTER(float, target_threshold)
        DECLARE_GETTER_SETTER(float, calculation_threshold)
        DECLARE_GETTER_SETTER(int, max_iterations)

        DECLARE_GETTER_SETTER(NodePath, end_effector_collider_path)
        DECLARE_GETTER_SETTER(float, end_effector_height_offset)
    };
}

#endif