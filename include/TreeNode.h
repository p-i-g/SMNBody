//
// Created by qiuzi on 7/2/2024.
//

#ifndef TREENODE_H
#define TREENODE_H

#include "GravityFunction.h"
#include "Particle.h"
#include "types.h"

class Sim;

class TreeNode {
    TreeNode *parent = nullptr;
    TreeNode *top_left = nullptr;
    TreeNode *top_right = nullptr;
    TreeNode *bottom_left = nullptr;
    TreeNode *bottom_right = nullptr;
    Sim* sim;

    sm_float size;

    Vector2 cm;

    Particle *particle = nullptr;
    bool has_children = false;
    sm_float charge = 0.;
    int num_particles = 0;

    void insert_into_child(Particle* particle, int depth) const; // this sounds so wrong
    void create_children();
    void resolve_collisions_downwards_recursion(Particle* particle, sm_float particle_radius, int* counters) const;

public:
    Vector2 center;
    TreeNode(TreeNode *parent, sm_float size, Vector2 center, Sim* sim);
    void insert_particle(Particle *particle, int depth);
    [[nodiscard]] Vector2 calculate_force(Particle* origin, sm_float opening_angle, GravityFunction& g, sm_float particle_radius) const;
    void resolve_collisions(Particle* particle, sm_float particle_radius, int* counters) const;
    ~TreeNode();
};
inline Vector2 avg = Vector2(0, 0);
inline sm_float avg_counter = 0;

#endif //TREENODE_H
