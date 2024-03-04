//
// Created by qiuzi on 29/2/2024.
//

#include "GravityFunction.h"
#include <cmath>
#include <iostream>

GravityFunction::GravityFunction(sm_float const capillary_length) : capillary_length(capillary_length){}


Vector2 GravityFunction::call(Vector2 const source, Vector2 const dest) const{
    sm_float const norm = (dest - source).norm();
    // if (std::cyl_bessel_k(1, norm / capillary_length) > 10) {
    //     std::cout << "norm: " << norm << std::endl;
    //     std::cout << "Bessel: " << std::cyl_bessel_k(1, norm / capillary_length) << std::endl;
    // }

    return (dest - source) * std::cyl_bessel_k(1, norm / capillary_length) / norm;
}
