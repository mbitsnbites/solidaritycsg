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

#ifndef _VOXELIZE_H_
#define _VOXELIZE_H_

#include "../Vector3.h"

namespace csg {

/// The Voxel primitive is an 8 bit signed integer.
typedef signed char Voxel;

/// Maximum voxel value
#define VOXEL_MAX 128


/// Voxelize class.
class Voxelize {
  public:
    /// Constructor.
    Voxelize();

    /// Destructor.
    virtual ~Voxelize() {}

    /// Define the voxel space boundaries and resolution.
    void SetVoxelSpace(float aMinX, float aMinY, float aMinZ,
                       float aMaxX, float aMaxY, float aMaxZ,
                       int aDivX, int aDivY, int aDivZ);

    /// Calculate a single slice of the voxel volume.
    /// The slice must be allocated by the caller, and hold DivX * DivY voxels.
    virtual void CalculateSlice(Voxel * aSlice, int aZ) = 0;

  private:
    Vector3 mMin; ///< Lower bound of the voxel space bounding box.
    Vector3 mMax; ///< Upper bound of the voxel space bounding box.
    int mDiv[3];  ///< Number of divisions of the voxel space (x, y, z).
};

}

#endif // _VOXELIZE_H_
