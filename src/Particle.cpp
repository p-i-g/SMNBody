//
// Created by qiuzi on 7/2/2024.
//

#include "../include/Particle.h"

#include <iostream>

Particle::Particle(Vector2 const position, Vector2 const velocity, sm_float const mass, sm_float const charge) :
position(position), velocity(velocity), mass(mass), charge(charge) {}

Particle::Particle(Vector2 const position, sm_float const mass, sm_float const charge) :
position(position), velocity(Vector2(0, 0)), mass(mass), charge(charge) {}

void Particle::resolve_collision(const Particle *other, sm_float const particle_radius) {
    // move this particle back
    Vector2 const diff = other->position - position;
    Vector2 prev_position = position;
    Vector2 prev_velocity = velocity;
    position = other->position - diff * 2 * particle_radius / diff.norm();

    // calculate the velocity for inelastic collision. its just an average of the 2 velocities
    velocity = (velocity + other->velocity) / 2;

    if (position.x > 10 || position.x < -10 || position.y > 10 || position.y < -10 ||
        velocity.x > 10 || velocity.x < -10 || velocity.y > 10 || velocity.y < -10) {
        std::cout << "New position: " << position.x << ", " << position.y << std::endl;
        std::cout << "Old position: " << prev_position.x << ", " << prev_position.y << std::endl;
        std::cout << "New velocity: " << velocity.x << ", " << velocity.y << std::endl;
        std::cout << "Old velocity: " << prev_velocity.x << ", " << prev_velocity.y << std::endl;
        std::cout << "Other velocity: " << other->velocity.x << ", " << other->velocity.y << std::endl;
    }
}
