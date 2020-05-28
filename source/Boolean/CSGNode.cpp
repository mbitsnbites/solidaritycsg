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

#include <stdexcept>
#include "CSGNode.h"
#include "../Array.h"

using namespace std;

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

namespace csg {

//-----------------------------------------------------------------------------
// CSGCompositeNode
//-----------------------------------------------------------------------------

CSGCompositeNode::~CSGCompositeNode() {
  // Recursively destroy all children
  list<CSGNode*>::iterator i;
  for (i = mChildren.begin(); i != mChildren.end(); ++i)
    delete (CSGNode*)(*i);
}

void CSGCompositeNode::SetSampleSpace(SampleSpace* aSampleSpace) {
  // Recursively set the sample space for all children
  list<CSGNode*>::iterator i;
  for (i = mChildren.begin(); i != mChildren.end(); ++i)
    (*i)->SetSampleSpace(aSampleSpace);

  // Keep a reference to the SampleSpace object
  mSampleSpace = aSampleSpace;
}

//-----------------------------------------------------------------------------
// CSGUnion
//-----------------------------------------------------------------------------

void CSGUnion::GetBoundingBox(BoundingBox& aAABB) {
  if (mChildren.empty()) {
    aAABB.mMin = aAABB.mMax = Vector3(0.0, 0.0, 0.0);
    return;
  }

  // Get the combined bounding box for all children
  bool first = true;
  list<CSGNode*>::iterator i;
  for (i = mChildren.begin(); i != mChildren.end(); ++i) {
    // Get child bounding box
    BoundingBox tmpAABB;
    (*i)->GetBoundingBox(tmpAABB);

    // Combine...
    if (first)
      aAABB = tmpAABB;
    else
      aAABB.Union(tmpAABB);

    first = false;
  }
}

bool CSGUnion::ComposeSlice(Voxel* aSlice, int aZ, int& aMinX, int& aMinY, int& aMaxX, int& aMaxY) {
  // Empty set?
  if (mChildren.size() == 0) {
    FillSlice(aSlice, -VOXEL_MAX, mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
    aMinX = aMinY = aMaxX = aMaxY = 0;
    return false;
  }

  bool first = true;
  list<CSGNode*>::iterator i;
  Array<Voxel> tmpSlice(mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
  for (i = mChildren.begin(); i != mChildren.end(); ++i) {
    if (first)
      first = !(*i)->ComposeSlice(aSlice, aZ, aMinX, aMinY, aMaxX, aMaxY);
    else {
      int minX, minY, maxX, maxY;
      if ((*i)->ComposeSlice(&tmpSlice[0], aZ, minX, minY, maxX, maxY)) {
        if (minX < aMinX)
          aMinX = minX;
        if (minY < aMinY)
          aMinY = minY;
        if (maxX > aMaxX)
          aMaxX = maxX;
        if (maxY > aMaxY)
          aMaxY = maxY;
        for (int y = minY; y <= maxY; ++y) {
          int idx = y * mSampleSpace->mDiv[0] + minX;
          for (int x = minX; x <= maxX; ++x) {
            aSlice[idx] = MAX(aSlice[idx], tmpSlice[idx]);
            ++idx;
          }
        }
      }
    }
  }

  return !first;
}

//-----------------------------------------------------------------------------
// CSGIntersection
//-----------------------------------------------------------------------------

void CSGIntersection::GetBoundingBox(BoundingBox& aAABB) {
  if (mChildren.empty()) {
    aAABB.mMin = aAABB.mMax = Vector3(0.0, 0.0, 0.0);
    return;
  }

  // Get the combined bounding box for all children
  bool first = true;
  list<CSGNode*>::iterator i;
  for (i = mChildren.begin(); i != mChildren.end(); ++i) {
    // Get child bounding box
    BoundingBox tmpAABB;
    (*i)->GetBoundingBox(tmpAABB);

    // Combine...
    if (first)
      aAABB = tmpAABB;
    else
      aAABB.Intersection(tmpAABB);

    first = false;
  }
}

bool CSGIntersection::ComposeSlice(Voxel* aSlice,
                                   int aZ,
                                   int& aMinX,
                                   int& aMinY,
                                   int& aMaxX,
                                   int& aMaxY) {
  // Empty set?
  if (mChildren.size() == 0) {
    FillSlice(aSlice, -VOXEL_MAX, mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
    aMinX = aMinY = aMaxX = aMaxY = 0;
    return false;
  }

  bool first = true;
  list<CSGNode*>::iterator i;
  Array<Voxel> tmpSlice(mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
  for (i = mChildren.begin(); i != mChildren.end(); ++i) {
    if (first) {
      if (!(*i)->ComposeSlice(aSlice, aZ, aMinX, aMinY, aMaxX, aMaxY))
        return false;
    } else {
      int minX, minY, maxX, maxY;
      bool nonEmpty = (*i)->ComposeSlice(&tmpSlice[0], aZ, minX, minY, maxX, maxY);

      // Empty result?
      if ((!nonEmpty) || (minX > aMaxX) || (maxX < aMinX) || (minY > aMaxY) || (maxY < aMinY)) {
        for (int y = aMinY; y <= aMaxY; ++y) {
          int idx = y * mSampleSpace->mDiv[0] + aMinX;
          for (int x = aMinX; x <= aMaxX; ++x) {
            aSlice[idx] = -VOXEL_MAX;
            ++idx;
          }
        }
        aMinX = aMinY = aMaxX = aMaxY = 0;
        return false;
      }

      // New bounding rectangle
      int newMinX = MAX(aMinX, minX);
      int newMaxX = MIN(aMaxX, maxX);
      int newMinY = MAX(aMinY, minY);
      int newMaxY = MIN(aMaxY, maxY);

      // Perform operation inside the new bounding rectangle, and fill with
      // "outside" in the remaining part of the old bounding rectangle
      for (int y = aMinY; y < newMinY; ++y) {
        int idx = y * mSampleSpace->mDiv[0] + aMinX;
        for (int x = aMinX; x <= aMaxX; ++x) {
          aSlice[idx] = -VOXEL_MAX;
          ++idx;
        }
      }
      for (int y = newMinY; y <= newMaxY; ++y) {
        int idx = y * mSampleSpace->mDiv[0] + aMinX;
        for (int x = aMinX; x < newMinX; ++x) {
          aSlice[idx] = -VOXEL_MAX;
          ++idx;
        }
        for (int x = newMinX; x <= newMaxX; ++x) {
          aSlice[idx] = MIN(aSlice[idx], tmpSlice[idx]);
          ++idx;
        }
        for (int x = newMaxX + 1; x <= aMaxX; ++x) {
          aSlice[idx] = -VOXEL_MAX;
          ++idx;
        }
      }
      for (int y = newMaxY + 1; y <= aMaxY; ++y) {
        int idx = y * mSampleSpace->mDiv[0] + aMinX;
        for (int x = aMinX; x <= aMaxX; ++x) {
          aSlice[idx] = -VOXEL_MAX;
          ++idx;
        }
      }

      // Use the new bounding rectangle
      aMinX = newMinX;
      aMaxX = newMaxX;
      aMinY = newMinY;
      aMaxY = newMaxX;
    }
    first = false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// CSGDifference
//-----------------------------------------------------------------------------

void CSGDifference::GetBoundingBox(BoundingBox& aAABB) {
  if (mChildren.empty()) {
    aAABB.mMin = aAABB.mMax = Vector3(0.0, 0.0, 0.0);
    return;
  }

  // For difference objects, the AABB is that of the first object
  (*mChildren.begin())->GetBoundingBox(aAABB);
}

bool CSGDifference::ComposeSlice(Voxel* aSlice,
                                 int aZ,
                                 int& aMinX,
                                 int& aMinY,
                                 int& aMaxX,
                                 int& aMaxY) {
  // Empty set?
  if (mChildren.size() == 0) {
    FillSlice(aSlice, -VOXEL_MAX, mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
    aMinX = aMinY = aMaxX = aMaxY = 0;
    return false;
  }

  bool first = true;
  list<CSGNode*>::iterator i;
  Array<Voxel> tmpSlice(mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
  for (i = mChildren.begin(); i != mChildren.end(); ++i) {
    if (first) {
      if (!(*i)->ComposeSlice(aSlice, aZ, aMinX, aMinY, aMaxX, aMaxY))
        return false;
    } else {
      int minX, minY, maxX, maxY;
      if ((*i)->ComposeSlice(&tmpSlice[0], aZ, minX, minY, maxX, maxY)) {
        if (aMinX > minX)
          minX = aMinX;
        if (aMinY > minY)
          minY = aMinY;
        if (aMaxX < maxX)
          maxX = aMaxX;
        if (aMaxY < maxY)
          maxY = aMaxY;
        if ((maxX >= minX) && (maxY >= minY)) {
          for (int y = minY; y <= maxY; ++y) {
            int idx = y * mSampleSpace->mDiv[0] + minX;
            for (int x = minX; x <= maxX; ++x) {
              aSlice[idx] = MIN(aSlice[idx], -tmpSlice[idx]);
              ++idx;
            }
          }
        }
      }
    }
    first = false;
  }

  return true;
}

//-----------------------------------------------------------------------------
// CSGShape
//-----------------------------------------------------------------------------

CSGShape::~CSGShape() {
  if (mVoxelize)
    delete mVoxelize;
}

Voxelize* CSGShape::DefineShape(Voxelize* aVoxelize) {
  if (mVoxelize)
    delete mVoxelize;
  mVoxelize = aVoxelize;
  return aVoxelize;
}

void CSGShape::SetSampleSpace(SampleSpace* aSampleSpace) {
  if (!mVoxelize)
    throw runtime_error("No shape defined.");

  mVoxelize->SetSampleSpace(aSampleSpace);
}

void CSGShape::GetBoundingBox(BoundingBox& aAABB) {
  if (!mVoxelize)
    throw runtime_error("No shape defined.");

  mVoxelize->GetBoundingBox(aAABB);
}

bool CSGShape::ComposeSlice(Voxel* aSlice, int aZ, int& aMinX, int& aMinY, int& aMaxX, int& aMaxY) {
  if (!mVoxelize)
    throw runtime_error("No shape defined.");

  // Calculate the slice
  return mVoxelize->CalculateSlice(aSlice, aZ, aMinX, aMinY, aMaxX, aMaxY);
}

}  // namespace csg
