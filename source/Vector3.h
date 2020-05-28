/*
  This file is part of SolidarityCSG.

  SolidarityCSG is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  SolidarityCSG is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SolidarityCSG.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _VECTOR3_H_
#define _VECTOR3_H_

#include <cmath>

namespace csg {

/// 3D vector class (double precision).
class Vector3 {
public:
  Vector3() {
    x = y = z = 0.0;
  }

  Vector3(double a, double b, double c) {
    x = a;
    y = b;
    z = c;
  }

  Vector3(float a, float b, float c) {
    x = a;
    y = b;
    z = c;
  }

  Vector3(const Vector3& a) {
    x = a.x;
    y = a.y;
    z = a.z;
  }

  inline double& operator[](unsigned int aIdx) const {
    return ((double*)&x)[aIdx];
  }

  inline Vector3 operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
  }

  inline Vector3 operator-(const Vector3& v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
  }

  inline Vector3 operator*(const double& aScale) const {
    return Vector3(aScale * x, aScale * y, aScale * z);
  }

  inline void operator+=(const Vector3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }

  inline bool operator==(const Vector3& v) const {
    return (x == v.x) && (y == v.y) && (z == v.z);
  }

  inline bool operator!=(const Vector3& v) const {
    return (x != v.x) || (y != v.y) || (z != v.z);
  }

  inline double Abs() {
    return sqrt(x * x + y * y + z * z);
  }

  inline Vector3 Normalize() {
    double s = Abs();
    if (s < 1e-50)
      return *this;
    s = 1.0 / s;
    return Vector3(x * s, y * s, z * s);
  }

  double x, y, z;
};

/// Compute the cross product of two vectors.
Vector3 Cross(Vector3& v1, Vector3& v2);

}  // namespace csg

#endif  // _VECTOR3_H_
