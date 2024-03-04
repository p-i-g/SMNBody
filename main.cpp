#include <iostream>

#include "Sim.h"
#include <cmath>

int main(int const argc, char** argv) {
    // std::cout << std::cyl_bessel_k(1, 0.0148148);
#pragma omp parallel for
    for (int i = 1; i < argc; i++) {
        const std::string params_file = argv[i];
        Sim sim(params_file);
        sim.run();
    }

    // std::cout << params_file << std::endl;
    return 0;
}
