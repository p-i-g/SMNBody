//
// Created by qiuzi on 7/2/2024.
//

#include "Vector2.h"

#include <cmath>

Vector2::Vector2(sm_float const x, sm_float const y) : x(x), y(y){}

Vector2 Vector2::operator *(sm_float const other) const {
    return {this->x * other, this->y * other};
}

Vector2 Vector2::operator /(sm_float const other) const {
    return {this->x / other, this->y / other};
}

Vector2 Vector2::operator +(Vector2 const other) const {
    return {this->x + other.x, this->y + other.y};
}

Vector2 Vector2::operator -(Vector2 const other) const {
    return {this->x - other.x, this->y - other.y};
}

sm_float Vector2::norm() const {
    return sqrt(pow(x, 2) + pow(y, 2));
}

