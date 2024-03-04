#include <iostream>

#include "Sim.h"
#include <cmath>

int main(int argc, char** argv) {
    // std::cout << std::cyl_bessel_k(1, 0.0148148);
    const std::string params_file = argv[1];
    // std::cout << params_file << std::endl;
    Sim sim(params_file);
    sim.run();
    return 0;
}
