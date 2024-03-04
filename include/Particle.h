//
// Created by qiuzi on 7/2/2024.
//

#ifndef PARTICLE_H
#define PARTICLE_H
#include "Vector2.h"

class TreeNode;

class Particle {
public:
    Vector2 position;
    Vector2 velocity;
    sm_float mass;
    sm_float charge;
    TreeNode* current_node = nullptr;

    Particle(Vector2 position, Vector2 velocity, sm_float mass, sm_float charge);
    Particle(Vector2 position, sm_float mass, sm_float charge);
    void resolve_collision(const Particle* other, sm_float particle_radius);
};



#endif //PARTICLE_H
