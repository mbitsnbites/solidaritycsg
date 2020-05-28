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

#ifndef _MESHVOXELIZE_H_
#define _MESHVOXELIZE_H_

#include "Voxelize.h"
#include "TriangleTree.h"
#include <vector>

namespace csg {

// Forward declarations
class Mesh;

/// Voxelize class for mesh objects.
class MeshVoxelize : public Voxelize {
public:
  /// Constructor.
  MeshVoxelize() : mAABBTree(0) {
  }

  /// Destructor.
  virtual ~MeshVoxelize() {
    if (mAABBTree)
      delete mAABBTree;
  }

  /// Define the triangle surface to be voxelized.
  /// A copy of the triangle surface is made internally in the Voxelize
  /// object, so the data arrays (aIndices, aVertices) can be free:d once this
  /// function has been called.
  void SetTriangles(int aTriangleCount, int* aIndices, int aVertexCount, double* aVertices);

  /// Define the triangle surface to be voxelized.
  /// A copy of the triangle surface is made internally in the Voxelize
  /// object, so the mesh object can be free:d once this function has been
  /// called.
  void SetTriangles(Mesh& aMesh);

  /// Calculate a single slice of the voxel volume. The slice must be
  /// allocated by the caller, and hold DivX * DivY voxels. The function
  /// returns false if all elements were cosidered "outside" (i.e. the slice
  /// is empty).
  virtual bool CalculateSlice(Voxel* aSlice,
                              int aZ,
                              int& aMinX,
                              int& aMinY,
                              int& aMaxX,
                              int& aMaxY);

private:
  /// Draw a single line segment (triangle/plane intersection) to the slice.
  void DrawLineSegment(Voxel* aSlice, Vector3& p1, Vector3& p2);

  /// Flood fill unvisited elements of the slice.
  void FloodFill(Voxel* aSlice, int aX, int aY, Voxel aValue);

  /// Triangles.
  std::vector<Triangle> mTriangles;

  /// Vertex coordinates.
  std::vector<Vector3> mVertices;

  /// Binary bounding box tree.
  AABBNode* mAABBTree;
};

}  // namespace csg

#endif  // _MESHVOXELIZE_H_
