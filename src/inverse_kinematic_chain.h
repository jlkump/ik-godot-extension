#ifndef INVERSE_KINEMATIC_CHAIN_H
#define INVERSE_KINEMATIC_CHAIN_H

#include <godot_cpp/classes/area3d.hpp>

#include "helpers.h"


namespace godot {
    class InverseKinematicChain : public Node {
        GDCLASS(InverseKinematicChain, Node)
    private:

        Array ik_joint_paths_;
        std::vector<Node3D*> ik_joints_;
        std::vector<Vector3> joints_; // Positions
        std::vector<float> distances_;
        Array constraint_mins_; // An array of Vector3, one for each joint which specifies the constraints on rotation
        Array constraint_maxs_; // Same as min, specifies constraint max for each joint. 

        NodePath root_pos_node_path_;
        Node3D* root_pos_node_;
        NodePath target_pos_path_;
        Node3D* target_pos_node_;
        float calculation_threshold_;
        float target_threshold_;
        int max_iterations_;


        Vector3 project_point_onto_line(Vector3 point, Vector3 line_dir, Vector3 line_pos);
        void perform_ik();
        // void update_bones();
        // void update_bone_vec_recursive(InverseKinematicBone* current);
        void update_joint_nodes();
        void update_joints();
        void update_distances();
        // void update_joints_and_distances();
        // Transform3D update_model_hierarchy_transform_recursive(Node3D* root, Node3D* cur);
        // void update_model_hierarchy_transform();

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

        DECLARE_GETTER_SETTER(Array, joint_paths)
        DECLARE_GETTER_SETTER(Array, constraint_mins)
        DECLARE_GETTER_SETTER(Array, constraint_maxs)
        DECLARE_GETTER_SETTER(NodePath, target_pos_path)
        DECLARE_GETTER_SETTER(NodePath, root_pos_node_path)

        DECLARE_GETTER_SETTER(float, target_threshold)
        DECLARE_GETTER_SETTER(float, calculation_threshold)
        DECLARE_GETTER_SETTER(int, max_iterations)
    };
}

#endif