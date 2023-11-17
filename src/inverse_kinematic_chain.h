#ifndef INVERSE_KINEMATIC_CHAIN_H
#define INVERSE_KINEMATIC_CHAIN_H

#include <godot_cpp/classes/area3d.hpp>

#include "helpers.h"


namespace godot {
    class InverseKinematicBone;

    class InverseKinematicChain : public Node {
        GDCLASS(InverseKinematicChain, Node)
    private:
        NodePath model_root_path_;
        Node3D* model_root_;
        Transform3D model_hierarchy_transform_;
        NodePath root_bone_path_;
        std::vector<InverseKinematicBone*> ik_bones_;
        std::vector<Vector3> joints_; // Positions
        std::vector<float> distances_;
        // std::vector<BoneConstraints*> constraints_; // TODO

        NodePath target_pos_path_;
        Node3D* target_pos_;
        float calculation_threshold_;
        float target_threshold_;
        int max_iterations_;


        void perform_ik();
        void update_bones();
        void update_bone_vec_recursive(InverseKinematicBone* current);
        void update_joints_and_distances();
        Transform3D update_model_hierarchy_transform_recursive(Node3D* root, Node3D* cur);
        void update_model_hierarchy_transform();

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

        DECLARE_GETTER_SETTER(NodePath, root_bone_path)
        DECLARE_GETTER_SETTER(NodePath, target_pos_path)
        DECLARE_GETTER_SETTER(NodePath, model_root_path)
        DECLARE_GETTER_SETTER(float, target_threshold)
        DECLARE_GETTER_SETTER(float, calculation_threshold)
        DECLARE_GETTER_SETTER(int, max_iterations)
    };
}

#endif