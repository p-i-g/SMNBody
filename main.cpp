#include <iostream>

#include "Sim.h"
#include <cmath>
#include <random>

int main(int const argc, char** argv) {
    // std::cout << std::cyl_bessel_k(1, 0.0148148);
    // Particle particle(Vector2(0, 0), 1, 1);
    //
    // std::random_device rd;
    // std::mt19937 gen{rd()};
    //
    // std::uniform_real_distribution<> start_distribution(0, 2 * std::numbers::pi);
    //
    // for (int i = 0; i < 10; i++) {
    //     sm_float const start = start_distribution(gen);
    //     sm_float const end = start + std::numbers::pi / 3 > 2 * std::numbers::pi ? start + std::numbers::pi / 3 - std::numbers::pi * 2 : start + std::numbers::pi / 3;
    //     std::cout << "(" << start << ", " << end << ")" << std::endl;
    //     std::cout << "-------------------------------------------" << std::endl;
    //     particle.add_lock(start, end);
    //
    //     for (int j = 0; j < 6; j++) {
    //         if (!std::isnan(particle.lock_start[j])) {
    //             std::cout << "(" << particle.lock_start[j] << ", " << particle.lock_end[j] << ")" << std::endl;
    //         }
    //     }
    //     std::cout << "===========================================" << std::endl;
    // }
    // return 0;

#pragma omp parallel for default(none) shared(argc, argv)
    for (int i = 1; i < argc; i++) {
        const std::string params_file = argv[i];
        Sim sim(params_file);
        sim.run();
    }

    // std::cout << params_file << std::endl;
    return 0;
}
