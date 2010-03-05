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

#include "BoundingBox.h"

namespace csg {

void BoundingBox::Union(BoundingBox &aBoundingBox)
{
  if(aBoundingBox.mMin.x < mMin.x)
    mMin.x = aBoundingBox.mMin.x;
  if(aBoundingBox.mMax.x > mMax.x)
    mMax.x = aBoundingBox.mMax.x;
  if(aBoundingBox.mMin.y < mMin.y)
    mMin.y = aBoundingBox.mMin.y;
  if(aBoundingBox.mMax.y > mMax.y)
    mMax.y = aBoundingBox.mMax.y;
  if(aBoundingBox.mMin.z < mMin.z)
    mMin.z = aBoundingBox.mMin.z;
  if(aBoundingBox.mMax.z > mMax.z)
    mMax.z = aBoundingBox.mMax.z;
}

void BoundingBox::Intersection(BoundingBox &aBoundingBox)
{
  if(aBoundingBox.mMin.x > mMin.x)
    mMin.x = aBoundingBox.mMin.x;
  if(aBoundingBox.mMax.x < mMax.x)
    mMax.x = aBoundingBox.mMax.x;
  if(aBoundingBox.mMin.y > mMin.y)
    mMin.y = aBoundingBox.mMin.y;
  if(aBoundingBox.mMax.y < mMax.y)
    mMax.y = aBoundingBox.mMax.y;
  if(aBoundingBox.mMin.z > mMin.z)
    mMin.z = aBoundingBox.mMin.z;
  if(aBoundingBox.mMax.z < mMax.z)
    mMax.z = aBoundingBox.mMax.z;

  // Did we get an empty bounding box (no overlap between boxes)?
  if((mMin.x >= mMax.x) || (mMin.y >= mMax.y) || (mMin.z > mMax.z))
    mMin = mMax = (mMin + mMax) * 0.5;
}

}
