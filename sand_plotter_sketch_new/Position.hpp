/* ****************************************************************************
Copyright 2025 k-off pacovali@student.42berlin.de

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
**************************************************************************** */

#pragma once

#include <math.h>

// Position: x,y
struct Pos {
  float x, y;
  Pos& operator-=(const Pos& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  friend Pos operator-(Pos lhs, const Pos& rhs) {
    return lhs -= rhs;
  }
  Pos& operator+=(const Pos& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  friend Pos operator+(Pos lhs, const Pos& rhs) {
    return lhs += rhs;
  }
  // scalar division
  template<class T> Pos& operator/=(const T& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  }
  // scalar division
  template<class T> friend Pos operator/(Pos p, const T& rhs) {
    return p /= rhs;
  }
  // scalar multiplication
  template<class T> Pos& operator*=(const T& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  }
  // scalar multiplication
  template<class T> friend Pos operator*(Pos p, const T& rhs) {
    return p *= rhs;
  }
  // Calculate radial distance (from 0,0 to this position)
  float radialDistance() const {
    return sqrt(x * x + y * y);
  }
};
