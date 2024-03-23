//
// Created by qiuzi on 7/2/2024.
//

#include "TreeNode.h"

#include <iostream>

#include "GravityFunction.h"
#include "Sim.h"

TreeNode::TreeNode(TreeNode *parent, sm_float const size, Vector2 const center, Sim* sim) :
parent(parent), size(size), center(center), cm(center), sim(sim) {}

void TreeNode::create_children() {
    if (!has_children) {
        top_left = new TreeNode(this, size / 2, center + Vector2(-size / 4, size / 4), sim);
        top_right = new TreeNode(this, size / 2, center + Vector2(size / 4, size / 4), sim);
        bottom_left = new TreeNode(this, size / 2, center + Vector2(-size / 4, -size / 4), sim);
        bottom_right = new TreeNode(this, size / 2, center + Vector2(size / 4, -size / 4), sim);
        has_children = true;
    }
    // std::cout << top_left->center.x << ", " << top_left->center.y << std::endl;
    // std::cout << top_right->center.x << ", " << top_right->center.y << std::endl;
    // std::cout << bottom_left->center.x << ", " << bottom_left->center.y << std::endl;
    // std::cout << bottom_right->center.x << ", " << bottom_right->center.y << std::endl;
    // std::cout << size << std::endl;
    has_children = true;
}

void TreeNode::insert_particle(Particle *particle, int depth) { // NOLINT it's meant to be recursive
    depth++;
    // check for out of bounds
    if (particle->position.x > center.x + size / 2 || particle->position.x < center.x - size / 2 ||
        particle->position.y > center.y + size / 2 || particle->position.y < center.y - size / 2) return;

    // std::cout << particle->position.x << ", " << particle->position.y << std::endl;
    // std::cout << center.x << ", " << center.y << std::endl;

    // update the aggregate things
    cm = (cm * charge + particle->position * particle->charge) / (charge + particle->charge);
    charge += particle->charge;

    if (num_particles > 0) {
        if (!has_children) {
            if (this->particle->position.x == particle->position.x && this->particle->position.y == particle->position.y) {
                particle->position = Vector2(particle->position.x + sim->epsilon_distribution(sim->gen), particle->position.y + sim->epsilon_distribution(sim->gen));
            }
            create_children();
        }
        insert_into_child(particle, depth);
        if (this->particle != nullptr) {
            insert_into_child(this->particle, depth);
            this->particle = nullptr;
        }
    } else {
        // std::cout << "Depth:" << depth << std::endl;
        this->particle = particle;
        particle->current_node = this;
    }
    num_particles++;
}

void TreeNode::insert_into_child(Particle *particle, int depth) const { // NOLINT it's meant to be recursive
    if (particle->position.x < center.x) {
        if (particle->position.y < center.y) {
            bottom_left->insert_particle(particle, depth);
        } else {
            top_left->insert_particle(particle, depth);
        }
    } else {
        if (particle->position.y < center.y) {
            bottom_right->insert_particle(particle, depth);
        } else {
            top_right->insert_particle(particle, depth);
        }
    }
}

Vector2 TreeNode::calculate_force(Particle* origin, sm_float const opening_angle, GravityFunction& g, sm_float const particle_radius) const { // NOLINT ik it's recursive
    if (sm_float const d = (origin->position - cm).norm(); size / d > opening_angle && has_children) { // the linter told me to do this. i don't know why
        // std::cout << size / d << std::endl;
        return
            top_left->calculate_force(origin, opening_angle, g, particle_radius) +
            top_right->calculate_force(origin, opening_angle, g, particle_radius) +
            bottom_left->calculate_force(origin, opening_angle, g, particle_radius) +
            bottom_right->calculate_force(origin, opening_angle, g, particle_radius);
    }
    if (origin != particle && (origin->position - cm).norm() > particle_radius * 2) {
        // if ((g.call(origin->position, cm) * charge).norm() / origin->mass > 10) {
        //     std::cout << g.call(origin->position, cm).norm() << ", " << charge << std::endl;
        // }
        avg = (avg * avg_counter + g.call(origin->position, cm) * charge) / (avg_counter + 1);
        avg_counter += 1;
        return g.call(origin->position, cm) * charge;
    }
    return {0, 0};
}

void TreeNode::resolve_collisions(Particle* particle, sm_float const particle_radius, int* counters) const { // NOLINT i know its recursive
    // traverse the tree upwards until we find a cell large enough such that there is no chance of collision with a particle outside this cell
    // ie boundary - particle.center > 2 * particle radius
    // std::cout << center.x << ", " << center.y << std::endl;
    // std::cout << particle->position.x << ", " << particle->position.y << std::endl;
    if ((center.x + size / 2 < particle->position.x + particle_radius * 2 || center.x - size / 2 > particle->position.x - particle_radius * 2 ||
         center.x + size / 2 < particle->position.x + particle_radius * 2 || center.x - size / 2 > particle->position.x - particle_radius * 2) &&
         parent != nullptr) {
        // recurse upwards
        parent->resolve_collisions(particle, particle_radius, counters);
    } else {
        resolve_collisions_downwards_recursion(particle, particle_radius, counters);
    }
}

void TreeNode::resolve_collisions_downwards_recursion(Particle* particle, sm_float const particle_radius, int* counters) const { // NOLINT i know its recursive
    if (has_children) {
        top_left->resolve_collisions_downwards_recursion(particle, particle_radius, counters);
        bottom_left->resolve_collisions_downwards_recursion(particle, particle_radius, counters);
        top_right->resolve_collisions_downwards_recursion(particle, particle_radius, counters);
        bottom_right->resolve_collisions_downwards_recursion(particle, particle_radius, counters);
    } else if (this->particle != nullptr && particle != this->particle &&
        (particle->position - this->particle->position).norm() < 2 * particle_radius) {
        particle->resolve_collision(this->particle, particle_radius, counters);
    }
}


TreeNode::~TreeNode() {
    delete top_left;
    delete top_right;
    delete bottom_left;
    delete bottom_right;
    if (particle != nullptr) particle->current_node = nullptr;
}
