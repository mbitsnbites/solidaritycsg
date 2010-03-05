/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _BOUNDINGBOX_H_
#define _BOUNDINGBOX_H_

#include "Vector3.h"

namespace csg {

/// Axis aligned bounding box class.
class BoundingBox {
  public:
    /// Get the diagonal of the bounding box.
    inline Vector3 Diagonal()
    {
      return mMax - mMin;
    }

    /// Calculate the union with another bounding box.
    void Union(BoundingBox &aBoundingBox);

    /// Calculate the intersection with another bounding box.
    void Intersection(BoundingBox &aBoundingBox);

    Vector3 mMin; ///< Lower bound of the bounding box.
    Vector3 mMax; ///< Upper bound of the bounding box.
};

}

#endif // _BOUNDINGBOX_H_
