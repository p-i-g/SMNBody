//
// Created by qiuzi on 7/2/2024.
//

#include "../include/Particle.h"

#include <iostream>

#include "Sim.h"

Particle::Particle(Vector2 const position, Vector2 const velocity, sm_float const mass, sm_float const charge,
                   Sim *sim) : position(position), velocity(velocity), mass(mass), charge(charge), sim(sim) {
}

Particle::Particle(Vector2 const position, sm_float const mass, sm_float const charge, Sim *sim) : position(position),
    velocity(Vector2(0, 0)), mass(mass), charge(charge), sim(sim) {
}

void Particle::resolve_collision(Particle *other, sm_float const particle_radius, int *counters) {
    // move this particle back
    Vector2 diff = other->position - position;
    // int retries = 0;
    // while (std::find(other->cache_angles.begin(), other->cache_angles.end(), atan2(-diff.y, -diff.x)) != other->cache_angles.end() ||
    //     diff.norm() == 0) {
    //     diff = Vector2(diff.x + sim->epsilon_distribution(sim->gen), diff.y + sim->epsilon_distribution(sim->gen));
    //     retries ++;
    // }
    // other->cache_angles.push_back(atan2(-diff.y, -diff.x));
    // cache_angles.push_back(atan2(diff.y, diff.x));

    if (diff.norm() != 0) {
        position = (other->position + position) / 2 - diff * particle_radius * 1.1 / diff.norm();
        other->position = position + diff * particle_radius * 2.2 / diff.norm();
    }

    // position = other->position - diff * 2 * particle_radius / diff.norm();

    // calculate the velocity for inelastic collision. its just an average of the 2 velocities
    // if (hehe_initialized) {
    //     hehe_before = hehe_before * 0.999 + velocity * 0.001;
    // } else {
    //     hehe_before = velocity;
    // }
    velocity = (velocity + other->velocity) / 2;
    other->velocity = velocity;
    // if (hehe_initialized) {
    //     hehe_after = hehe_after * 0.999 + velocity * 0.001;
    // } else {
    //     hehe_after = velocity;
    //     hehe_initialized = true;
    // }

    // if (position.x > 10 || position.x < -10 || position.y > 10 || position.y < -10 ||
    //     velocity.x > 10 || velocity.x < -10 || velocity.y > 10 || velocity.y < -10) {
    //     std::cout << "New position: " << position.x << ", " << position.y << std::endl;
    //     std::cout << "Old position: " << prev_position.x << ", " << prev_position.y << std::endl;
    //     std::cout << "New velocity: " << velocity.x << ", " << velocity.y << std::endl;
    //     std::cout << "Old velocity: " << prev_velocity.x << ", " << prev_velocity.y << std::endl;
    //     std::cout << "Other velocity: " << other->velocity.x << ", " << other->velocity.y << std::endl;
    // }
    sm_float const self_angle = atan2(-diff.y, -diff.x) + std::numbers::pi;
    sm_float const self_start = self_angle - std::numbers::pi / 6 >= 0
                                    ? self_angle - std::numbers::pi / 6
                                    : self_angle - std::numbers::pi / 6 + 2 * std::numbers::pi;
    sm_float const self_end = self_angle + std::numbers::pi / 6 > 2 * std::numbers::pi
                                  ? self_angle + std::numbers::pi / 6 - 2 * std::numbers::pi
                                  : self_angle + std::numbers::pi / 6;
    sm_float const other_angle = atan2(diff.y, diff.x) + std::numbers::pi;
    sm_float const other_start = other_angle - std::numbers::pi / 6 >= 0
                                     ? other_angle - std::numbers::pi / 6
                                     : other_angle - std::numbers::pi / 6 + 2 * std::numbers::pi;
    sm_float const other_end = other_angle + std::numbers::pi / 6 > 2 * std::numbers::pi
                                   ? other_angle + std::numbers::pi / 6 - 2 * std::numbers::pi
                                   : other_angle + std::numbers::pi / 6;
    // for (int k = static_cast<int>(self_start * 100000 / 2 / std::numbers::pi);
    //      k < static_cast<int>(self_end * 100000 / 2 / std::numbers::pi) && k < 100000; k++) {
    //     counters[k]++;
    //      }
    // for (int k = static_cast<int>(other_start * 100000 / 2 / std::numbers::pi);
    //      k < static_cast<int>(other_end * 100000 / 2 / std::numbers::pi) && k < 100000; k++) {
    //     counters[k]++;
    //      }
    // std::cout << "Self: " << self_angle << ", " << self_start << ", " << self_end << std::endl;
    // std::cout << "Other: " << other_angle << ", " << other_start << ", " << other_end << std::endl;
    add_lock(self_start, self_end);
    other->add_lock(other_start, other_end);
}


void Particle::add_lock(sm_float start, sm_float end) {
    if (start < 0 || start > 2 * std::numbers::pi || end < 0 || end > 2 * std::numbers::pi) {
        std::cout << "fuck" << std::endl;
    }
    if (start > end) {
        end = end + 2 * std::numbers::pi;
    }

    bool overlap[6] = {false, false, false, false, false, false};

    for (int i = 0; i < 6; i++) {
        if (std::isnan(lock_start[i])) {
            overlap[i] = true;
            continue;;
        }
        sm_float const temp_start = lock_start[i];
        sm_float temp_end = lock_end[i];

        if (temp_start > temp_end) {
            temp_end = temp_end + 2 * std::numbers::pi;
        }

        if (temp_start < start && temp_end > start) {
            overlap[i] = true;
            start = temp_start;
        }
        if (temp_end > end && temp_start < end) {
            overlap[i] = true;
            end = temp_end;
        }
    }

    bool added = false;

    for (int i = 0; i < 6; i++) {
        if (overlap[i]) {
            // std::cout << added << std::endl;
            if (!added) {
                lock_start[i] = start;

                if (end > 2 * std::numbers::pi) {
                    lock_end[i] = end - 2 * std::numbers::pi;
                } else {
                    lock_end[i] = end;
                }
                added = true;
            } else {
                lock_start[i] = nan("1");
                lock_end[i] = nan("1");
                // std::cout << std::isnan(lock_start[i]);
            }
        }
    }
}

void Particle::clear_lock() {
    for (int i = 0; i < 6; i++) {
        lock_start[i] = nan("1");
        lock_end[i] = nan("1");
    }
}

Vector2 Particle::check_lock(Vector2 const force) const {
    sm_float const angle = atan2(-force.y, -force.x) + std::numbers::pi;
    for (int i = 0; i < 6; i++) {
        if (!std::isnan(lock_start[i])) {
            if (lock_start[i] < angle && angle < lock_end[i]) {
                return {0, 0};
            } else if (lock_start[i] > lock_end[i] && lock_start[i] < angle || lock_end[i] > angle) {
                return {0, 0};
            }
        }
    }
    return force;
}
