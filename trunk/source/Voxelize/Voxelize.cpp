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

#include "Voxelize.h"

namespace csg {


//-----------------------------------------------------------------------------
// Voxelize
//-----------------------------------------------------------------------------

Voxelize::Voxelize()
{
  // FIXME!
}

Voxelize::~Voxelize()
{
  // FIXME!
}

void Voxelize::SetVoxelSpace(float aMinX, float aMinY, float aMinZ, float aMaxX,
  float aMaxY, float aMaxZ, int aDivX, int aDivY, int aDivZ)
{
  mMin = Vector3(aMinX, aMinY, aMinZ);
  mMax = Vector3(aMaxX, aMaxY, aMaxZ);
  mDiv[0] = aDivX;
  mDiv[1] = aDivY;
  mDiv[2] = aDivZ;
}

}
