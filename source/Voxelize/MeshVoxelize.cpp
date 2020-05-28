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

#include "MeshVoxelize.h"
#include "../FileIO/Mesh.h"
#include "../Array.h"
#include <stdexcept>

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// MeshVoxelize
//-----------------------------------------------------------------------------

void MeshVoxelize::SetTriangles(Mesh& aMesh) {
  SetTriangles(int(aMesh.mIndices.size() / 3),
               &aMesh.mIndices[0],
               int(aMesh.mVertices.size()),
               &aMesh.mVertices[0].x);
}

void MeshVoxelize::SetTriangles(int aTriangleCount,
                                int* aIndices,
                                int aVertexCount,
                                double* aVertices) {
  // Sanity check
  if ((aTriangleCount < 4) || (!aIndices) || (aVertexCount < 4) || (!aVertices))
    throw runtime_error("Invalid mesh definition.");

  // Create vertex array, and calculate the shape bounding box
  mVertices.resize(aVertexCount);
  double* vPtr = aVertices;
  mAABB.mMin = mAABB.mMax = Vector3(vPtr[0], vPtr[1], vPtr[2]);
  for (int i = 0; i < aVertexCount; ++i) {
    // Copy vertex coordinates
    mVertices[i] = Vector3(vPtr[0], vPtr[1], vPtr[2]);

    // Update bounding box min/max
    if (mVertices[i].x < mAABB.mMin.x)
      mAABB.mMin.x = mVertices[i].x;
    else if (mVertices[i].x > mAABB.mMax.x)
      mAABB.mMax.x = mVertices[i].x;
    if (mVertices[i].y < mAABB.mMin.y)
      mAABB.mMin.y = mVertices[i].y;
    else if (mVertices[i].y > mAABB.mMax.y)
      mAABB.mMax.y = mVertices[i].y;
    if (mVertices[i].z < mAABB.mMin.z)
      mAABB.mMin.z = mVertices[i].z;
    else if (mVertices[i].z > mAABB.mMax.z)
      mAABB.mMax.z = mVertices[i].z;

    vPtr += 3;
  }

  // Create triangle array
  mTriangles.resize(aTriangleCount);
  int* iPtr = aIndices;
  for (int i = 0; i < aTriangleCount; ++i) {
    mTriangles[i].SetCoordinates(&mVertices[iPtr[0]], &mVertices[iPtr[1]], &mVertices[iPtr[2]]);
    iPtr += 3;
  }

  // Build the bounding box tree
  mAABBTree = BuildAABBTree(mTriangles);
}

bool MeshVoxelize::CalculateSlice(Voxel* aSlice,
                                  int aZ,
                                  int& aMinX,
                                  int& aMinY,
                                  int& aMaxX,
                                  int& aMaxY) {
  // Check that the voxel space has been properly set up
  if (!mSampleSpace || !mSampleSpace->IsValid())
    throw runtime_error("Undefined/invalid voxel space dimensions.");

  // Check that the mesh has been properly set up
  if (!mAABBTree)
    throw runtime_error("Undefined triangle mesh.");

  // Calculate the slice plane Z value
  Vector3 voxelSize = mSampleSpace->VoxelSize();
  double planeZ = aZ * voxelSize.z + mSampleSpace->mAABB.mMin.z;

  // Get all intersecting triangles
  list<Triangle*> triList;
  mAABBTree->IntersectingTriangles(planeZ, triList);

  // Keep track of the bounding rectangle for the intersection in this slice
  double min[2], max[2];
  min[0] = mSampleSpace->mAABB.mMax.x + 4.0 * voxelSize.x;
  min[1] = mSampleSpace->mAABB.mMax.y + 4.0 * voxelSize.y;
  max[0] = mSampleSpace->mAABB.mMin.x - 4.0 * voxelSize.x;
  max[1] = mSampleSpace->mAABB.mMin.y - 4.0 * voxelSize.y;

  // Calculate all intersections with the slice, and get the bounding rectangle
  // of the intersection
  Array<Vector3> intersections(triList.size() * 2);
  int count = 0;
  Vector3* vPtr = &intersections[0];
  for (list<Triangle*>::iterator t = triList.begin(); t != triList.end(); ++t) {
    if ((*t)->IntersectPlane(planeZ, &vPtr[0], &vPtr[1])) {
      for (int i = 0; i < 2; ++i) {
        if (vPtr[i].x < min[0])
          min[0] = vPtr[i].x;
        if (vPtr[i].y < min[1])
          min[1] = vPtr[i].y;
        if (vPtr[i].x > max[0])
          max[0] = vPtr[i].x;
        if (vPtr[i].y > max[1])
          max[1] = vPtr[i].y;
      }
      vPtr += 2;
      ++count;
    }
  }

  // Convert bounding rectangle to voxel indices
  aMinX = int((min[0] - mSampleSpace->mAABB.mMin.x) / voxelSize.x) - 2;
  aMinY = int((min[1] - mSampleSpace->mAABB.mMin.y) / voxelSize.y) - 2;
  aMaxX = int((max[0] - mSampleSpace->mAABB.mMin.x) / voxelSize.x) + 2;
  aMaxY = int((max[1] - mSampleSpace->mAABB.mMin.y) / voxelSize.y) + 2;

  // Does the mesh volume miss this slice completely?
  if ((count == 0) || (aMaxX < 0) || (aMaxY < 0) || (aMinX >= mSampleSpace->mDiv[0]) ||
      (aMinY >= mSampleSpace->mDiv[1])) {
    FillSlice(aSlice, -VOXEL_MAX, mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1]);
    aMinX = aMinY = aMaxX = aMaxY = 0;
    return false;
  }

  // Clamp first/last indices of the bounding rectangle
  if (aMinX < 0)
    aMinX = 0;
  if (aMinY < 0)
    aMinY = 0;
  if (aMaxX >= mSampleSpace->mDiv[0])
    aMaxX = mSampleSpace->mDiv[0] - 1;
  if (aMaxY >= mSampleSpace->mDiv[1])
    aMaxY = mSampleSpace->mDiv[1] - 1;

  // Mark all voxels inside the intersection bounding rectangle as "UNVISITED",
  // and all voxels outside of the intersection bounding rectangle as "outside".
  Voxel* ptr = aSlice;
  for (int y = 0; y < aMinY; ++y) {
    for (int x = 0; x < mSampleSpace->mDiv[0]; ++x)
      *ptr++ = -VOXEL_MAX;
  }
  for (int y = aMinY; y <= aMaxY; ++y) {
    for (int x = 0; x < aMinX; ++x)
      *ptr++ = -VOXEL_MAX;
    for (int x = aMinX; x <= aMaxX; ++x)
      *ptr++ = VOXEL_UNVISITED;
    for (int x = aMaxX + 1; x < mSampleSpace->mDiv[0]; ++x)
      *ptr++ = -VOXEL_MAX;
  }
  for (int y = aMaxY + 1; y < mSampleSpace->mDiv[1]; ++y) {
    for (int x = 0; x < mSampleSpace->mDiv[0]; ++x)
      *ptr++ = -VOXEL_MAX;
  }

  // Draw all intersecting triangles as line segments to the slice
  for (int i = 0; i < count; ++i)
    DrawLineSegment(aSlice, intersections[i * 2], intersections[i * 2 + 1]);

  // Determine in/out for all the visited (=drawn) voxels
  for (int y = aMinY; y <= aMaxY; ++y) {
    ptr = &aSlice[y * mSampleSpace->mDiv[0] + aMinX];
    for (int x = aMinX; x <= aMaxX; ++x) {
      Voxel value = *ptr;
      if (value != VOXEL_UNVISITED) {
        // Calculate voxel 3D coordinate
        Vector3 p;
        p.x = x * voxelSize.x + mSampleSpace->mAABB.mMin.x;
        p.y = y * voxelSize.y + mSampleSpace->mAABB.mMin.y;
        p.z = planeZ;

        // Point outside? If so, flip sign...
        if (!mAABBTree->PointInside(p))
          *ptr = -value;
      }
      ++ptr;
    }
  }

  // Flood fill the unvisited parts of the slice according to in/out
  for (int y = aMinY; y <= aMaxY; ++y) {
    ptr = &aSlice[y * mSampleSpace->mDiv[0] + aMinX];
    for (int x = aMinX; x <= aMaxX; ++x) {
      if (*ptr == VOXEL_UNVISITED) {
        // Calculate voxel 3D coordinate
        Vector3 p;
        p.x = x * voxelSize.x + mSampleSpace->mAABB.mMin.x;
        p.y = y * voxelSize.y + mSampleSpace->mAABB.mMin.y;
        p.z = planeZ;

        // Check if the voxel is inside or outside of the triangle mesh
        Voxel value = mAABBTree->PointInside(p) ? VOXEL_MAX : -VOXEL_MAX;

        // Flood fill from this voxel
        FloodFill(aSlice, x, y, value);
      }
      ++ptr;
    }
  }

  return true;
}

/// Helper funciton for updating the voxel values (used by DrawLineSegment).
static void UpdateVoxelValues(Voxel* aSlice, int aSliceSize, int aIdx1, int aIdx2, double& aDist1) {
  if ((aIdx1 >= 0) && (aIdx1 < aSliceSize)) {
    Voxel v1 = (Voxel)(aDist1 * VOXEL_MAX + 0.5);
    if ((aSlice[aIdx1] == VOXEL_UNVISITED) || (v1 < aSlice[aIdx1]))
      aSlice[aIdx1] = v1;
  }
  if ((aIdx2 >= 0) && (aIdx2 < aSliceSize)) {
    Voxel v2 = (Voxel)((1.0 - aDist1) * VOXEL_MAX + 0.5);
    if ((aSlice[aIdx2] == VOXEL_UNVISITED) || (v2 < aSlice[aIdx2]))
      aSlice[aIdx2] = v2;
  }
}

void MeshVoxelize::DrawLineSegment(Voxel* aSlice, Vector3& p1, Vector3& p2) {
  // Convert the coordinates from world space to voxel index space
  double scaleX = mSampleSpace->mDiv[0] / (mSampleSpace->mAABB.mMax.x - mSampleSpace->mAABB.mMin.x);
  double scaleY = mSampleSpace->mDiv[1] / (mSampleSpace->mAABB.mMax.y - mSampleSpace->mAABB.mMin.y);
  double x1 = (p1.x - mSampleSpace->mAABB.mMin.x) * scaleX;
  double y1 = (p1.y - mSampleSpace->mAABB.mMin.y) * scaleY;
  double x2 = (p2.x - mSampleSpace->mAABB.mMin.x) * scaleX;
  double y2 = (p2.y - mSampleSpace->mAABB.mMin.y) * scaleY;

  int sliceSize = mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1];

  // Calculate direction, and determine major axis
  double dx = x2 - x1;
  double dy = y2 - y1;
  if (fabs(dx) > fabs(dy)) {
    // Make sure that the step along the major axis is positive
    if (dx < 0.0) {
      double tmp = x1;
      x1 = x2;
      x2 = tmp;
      tmp = y1;
      y1 = y2;
      y2 = tmp;
      dx = -dx;
      dy = -dy;
    }

    // Step length along minor axis (when major axis step length = 1.0)
    dy /= dx;
    double dyInv = 1.0 / dy;

    // Calculate starting point (first intersection along the minor axis)
    double x, y, fracX;
    fracX = modf(x1, &x);
    if (fracX < 0.0)
      fracX += 1.0;
    else
      x += 1.0;
    y = y1 + (1.0 - fracX) * dy;

    // Loop and process all edge crossings
    int yIntOld = int(floor(y1));
    int xInt = int(floor(x));
    int numMajorCrossings = int(ceil(x2)) - xInt + 1;
    for (int i = 0; i < numMajorCrossings; ++i) {
      // Get integer voxel coordinates, and check if we are within bounds
      int yInt = int(floor(y));
      if ((xInt >= 0) && (xInt < mSampleSpace->mDiv[0]) && (yInt >= 0) &&
          (yInt < mSampleSpace->mDiv[1])) {
        int idx1 = yInt * mSampleSpace->mDiv[0] + xInt;

        // Did we have a crossing along the minor axis?
        if (yInt != yIntOld) {
          // Calculate the crossing along the major axis
          double d;
          if (dy > 0.0) {
            d = (y - double(yInt)) * dyInv;
            UpdateVoxelValues(aSlice, sliceSize, idx1, idx1 - 1, d);
          } else {
            d = (y - double(yIntOld)) * dyInv;
            UpdateVoxelValues(aSlice,
                              sliceSize,
                              idx1 + mSampleSpace->mDiv[0],
                              idx1 + mSampleSpace->mDiv[0] - 1,
                              d);
          }
        }

        // Calculate the crossing along the minor axis (one per step)
        double d = y - double(yInt);

        // Update the voxel values
        UpdateVoxelValues(aSlice, sliceSize, idx1, idx1 + mSampleSpace->mDiv[0], d);
      }

      // Next crossing along the major axis
      yIntOld = yInt;
      y += dy;
      ++xInt;
    }
  } else {
    // Make sure that the step along the major axis is positive
    if (dy < 0.0) {
      double tmp = x1;
      x1 = x2;
      x2 = tmp;
      tmp = y1;
      y1 = y2;
      y2 = tmp;
      dx = -dx;
      dy = -dy;
    }

    // Step length along minor axis (when major axis step length = 1.0)
    dx /= dy;
    double dxInv = 1.0 / dx;

    // Calculate starting point (first intersection along the minor axis)
    double x, y, fracY;
    fracY = modf(y1, &y);
    if (fracY < 0.0)
      fracY += 1.0;
    else
      y += 1.0;
    x = x1 + (1.0 - fracY) * dx;

    // Loop and process all edge crossings
    int xIntOld = int(floor(x1));
    int yInt = int(floor(y));
    int numMajorCrossings = int(ceil(y2)) - yInt + 1;
    for (int i = 0; i < numMajorCrossings; ++i) {
      // Get integer voxel coordinates, and check if we are within bounds
      int xInt = int(floor(x));
      if ((xInt >= 0) && (xInt < mSampleSpace->mDiv[0]) && (yInt >= 0) &&
          (yInt < mSampleSpace->mDiv[1])) {
        int idx1 = yInt * mSampleSpace->mDiv[0] + xInt;

        // Did we have a crossing along the minor axis?
        if (xInt != xIntOld) {
          // Calculate the crossing along the major axis
          double d;
          if (dx > 0.0) {
            d = (x - double(xInt)) * dxInv;
            UpdateVoxelValues(aSlice, sliceSize, idx1, idx1 - mSampleSpace->mDiv[0], d);
          } else {
            d = (x - double(xIntOld)) * dxInv;
            UpdateVoxelValues(aSlice, sliceSize, idx1 + 1, idx1 + 1 - mSampleSpace->mDiv[0], d);
          }
        }

        // Calculate the crossing along the minor axis (one per step)
        double d = x - double(xInt);

        // Update the voxel values
        UpdateVoxelValues(aSlice, sliceSize, idx1, idx1 + 1, d);
      }

      // Next crossing along the major axis
      xIntOld = xInt;
      x += dx;
      ++yInt;
    }
  }
}

void MeshVoxelize::FloodFill(Voxel* aSlice, int aX, int aY, Voxel aValue) {
  // Sanity check
  if ((aValue == VOXEL_UNVISITED) || (aX < 0) || (aX >= mSampleSpace->mDiv[0]) || (aY < 0) ||
      (aY >= mSampleSpace->mDiv[1]))
    throw runtime_error("Invalid fill operation.");

  // Get index limits for this slice
  int xCount = mSampleSpace->mDiv[0];
  int yCount = mSampleSpace->mDiv[1];
  int xMax = xCount - 1;
  int sliceSize = xCount * yCount;

  // Initialize the fill queue with the starting point
  list<int> queue;
  queue.push_back(aY * xCount + aX);

  // Flood...
  while (!queue.empty()) {
    // Get next voxel to evaluate
    int idx = queue.front();
    queue.pop_front();

    // Convert 1D index to the corresponding x coordinate
    int x = idx % xCount;

    // Find first non-filled voxel along this column
    while ((idx >= 0) && (aSlice[idx] == VOXEL_UNVISITED))
      idx -= xCount;
    idx += xCount;

    // Fill this column as far as its unvisited
    bool spanLeft = false;
    bool spanRight = false;
    while ((idx < sliceSize) && (aSlice[idx] == VOXEL_UNVISITED)) {
      // Set fill value
      aSlice[idx] = aValue;

      // Append neighbours to the fill queue
      if ((!spanLeft) && (x > 0) && (aSlice[idx - 1] == VOXEL_UNVISITED)) {
        queue.push_back(idx - 1);
        spanLeft = true;
      } else if (spanLeft && (x > 0) && (aSlice[idx - 1] != VOXEL_UNVISITED)) {
        spanLeft = false;
      }
      if ((!spanRight) && (x < xMax) && (aSlice[idx + 1] == VOXEL_UNVISITED)) {
        queue.push_back(idx + 1);
        spanRight = true;
      } else if (spanRight && (x < xMax) && (aSlice[idx + 1] != VOXEL_UNVISITED)) {
        spanRight = false;
      }

      idx += xCount;
    }
  }
}

}  // namespace csg
