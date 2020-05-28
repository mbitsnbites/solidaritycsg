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

#include "Curver.h"
#include <cmath>

using namespace std;

namespace csg {

//------------------------------------------------------------------------------
// Point
//------------------------------------------------------------------------------

/// Normalize a point.
static Point Normalize(const Point& aPoint) {
  double len = aPoint.x * aPoint.x + aPoint.y * aPoint.y;
  if (len > 1e-100) {
    double lenInv = 1.0 / sqrt(len);
    return Point(aPoint.x * lenInv, aPoint.y * lenInv);
  } else
    return Point(0.0, 0.0);
}

/// Calculate the angle from v1 to v2.
/// The angle difference is positive for clockwise rotation and negative for
/// counter clockwise rotation.
/// @note v1 and v2 need to be normalized.
static double CalcDeltaAngle(const Point& v1, const Point& v2) {
  double deltaAngle = acos(v1.x * v2.x + v1.y * v2.y);
  if ((v1.x * v2.y - v1.y * v2.x) > 0.0)
    deltaAngle = -deltaAngle;
  return deltaAngle;
}

//------------------------------------------------------------------------------
// Contour
//------------------------------------------------------------------------------

bool Contour::IsExterior() {
  if (mPoints.size() < 3)
    return true;

  // Starting vector
  Point v2 = Normalize(mPoints[0] - mPoints[mPoints.size() - 1]);

  // Calculate Accumulated angle
  double angle = 0.0;
  for (unsigned int i = 1; i < mPoints.size(); ++i) {
    Point v1 = v2;
    v2 = Normalize(mPoints[i] - mPoints[i - 1]);
    angle += CalcDeltaAngle(v1, v2);
  }

  // The contour is considered exterior if it has clockwise orientation
  return (angle >= 0.0);
}

//------------------------------------------------------------------------------
// Curver
//------------------------------------------------------------------------------

Curver::~Curver() {
  // Free all contours
  for (list<Contour*>::iterator i = mContours.begin(); i != mContours.end(); ++i)
    delete *i;
}

void Curver::MakeContours(Voxel* aSlice) {
  // FIXME!
}

}  // namespace csg
