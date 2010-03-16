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

#ifndef _SPHEREVOXELIZE_H_
#define _SPHEREVOXELIZE_H_

#include "Voxelize.h"

namespace csg {

/// Voxelize class for sphere objects.
class SphereVoxelize : public Voxelize {
  public:
    /// Constructor.
    SphereVoxelize() {}

    /// Destructor.
    virtual ~SphereVoxelize() {}

    /// Define the sphere to be voxelized.
    void SetSphere(Vector3 aCenter, double aRadius);

    /// Calculate a single slice of the voxel volume.
    /// The slice must be allocated by the caller, and hold DivX * DivY voxels.
    virtual void CalculateSlice(Voxel * aSlice, int aZ);

  private:
    Vector3 mCenter; ///< Sphere center.
    double mRadius;  ///< Sphere radius.
};

}

#endif // _SPHEREVOXELIZE_H_