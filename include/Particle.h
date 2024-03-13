//
// Created by qiuzi on 7/2/2024.
//

#ifndef PARTICLE_H
#define PARTICLE_H
#include <cmath>
#include <vector>

#include "Vector2.h"

class TreeNode;
class Sim;

class Particle {
public:
    Vector2 position;
    Vector2 velocity;
    sm_float mass;
    sm_float charge;
    TreeNode* current_node = nullptr;
    sm_float lock_start[6] = {nan("1"), nan("1"), nan("1"), nan("1"), nan("1"), nan("1")};
    sm_float lock_end[6] = {nan("1"), nan("1"), nan("1"), nan("1"), nan("1"), nan("1")};
    std::vector<sm_float> cache_angles = std::vector<sm_float>();
    Sim* sim;

    Particle(Vector2 position, Vector2 velocity, sm_float mass, sm_float charge, Sim* sim);
    Particle(Vector2 position, sm_float mass, sm_float charge, Sim* sim);
    void resolve_collision(Particle* other, sm_float particle_radius);
    void add_lock(sm_float start, sm_float end);
    void clear_lock();
    [[nodiscard]] Vector2 check_lock(Vector2 force) const;
};

// inline Vector2 hehe_before = Vector2(0, 0);
// inline Vector2 hehe_after = Vector2(0, 0);
// inline bool hehe_initialized = false;



#endif //PARTICLE_H
