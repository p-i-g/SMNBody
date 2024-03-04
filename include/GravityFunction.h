//
// Created by qiuzi on 29/2/2024.
//

#ifndef GRAVITYFUNCTION_H
#define GRAVITYFUNCTION_H
#include "Vector2.h"


class GravityFunction {
    sm_float capillary_length;

public:
    explicit GravityFunction(sm_float capillary_length);
    [[nodiscard]] Vector2 call(Vector2 source, Vector2 dest) const;
};


#endif //GRAVITYFUNCTION_H
