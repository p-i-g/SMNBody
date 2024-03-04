//
// Created by qiuzi on 7/2/2024.
//

#ifndef VECTOR2_H
#define VECTOR2_H
#include "types.h"


class Vector2 {
    /**
     * Represents a vector in 2D space.
     */

public:
    sm_float x = 0.;
    sm_float y = 0.;

    Vector2(sm_float x, sm_float y);
    Vector2 operator +(Vector2 other) const;
    Vector2 operator -(Vector2 other) const;
    Vector2 operator *(sm_float other) const;
    Vector2 operator /(sm_float other) const;
    [[nodiscard]] sm_float norm() const;
};



#endif //VECTOR2_H
