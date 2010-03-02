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

#include <vector>
#include "Vector3.h"

namespace csg {

/// The Voxel primitive is an 8 bit unsigned integer.
typedef unsigned char Voxel;

/// Triangle class.
class Triangle {
  public:
    /// Define the triangle. p1, p2 and p3 are pointers to the vertex
    /// coordinates for the three corners of the triangle.
    void SetCoordinates(Vector3 * p1, Vector3 * p2, Vector3 * p3);

    /// Calculate the intersection with an XY-plane.
    /// The result of the intersection, if any, is a line segment that is
    /// returned in p1 and p2.
    bool IntersectPlane(double aPlaneZ, Vector3 * p1, Vector3 * p2);

    /// Calculate the intersection with a Z-ray.
    /// Only positive intersections are considered.
    /// The result of the intersection is the t-value (offset) along the ray for
    /// the intersection. A negative return value indicates that there was no
    /// intersection.
    double IntersectZRay(Vector3 aOrigin);

  private:
    Vector3 * mVertices[3]; ///< Pointers to the actual vertex coordinates.
    Vector3 mMin;           ///< Lower bound of the triangle AABB.
    Vector3 mMax;           ///< Upper bound of the triangle AABB.
    Vector3 mNormal;        ///< Triangle normal (out direction).
};

/// Voxelize class.
class Voxelize {
  public:
    /// Constructor.
    Voxelize();

    /// Destructor.
    ~Voxelize();

    /// Define the triangle surface to be voxelized.
    /// A copy of the triangle surface is made internally in the Voxelize
    /// object, so the data arrays (aIndices, aVertices) can be free:d once this
    /// function has been called.
    void SetTriangles(int aTriangleCount, int * aIndices, int aVertexCount,
                      float * aVertices);

    /// Define the voxel space boundaries and resolution.
    void SetVoxelSpace(float aMinX, float aMinY, float aMinZ,
                       float aMaxX, float aMaxY, float aMaxZ,
                       int aDivX, int aDivY, int aDivZ);

    /// Calculate a single slice of the voxel volume.
    /// The slice must be allocated by the caller, and hold DivX * DivY voxels.
    void CalculateSlice(Voxel * aSlice, int aZ);

  private:
    Vector3 mMin; ///< Lower bound of the voxel space bounding box.
    Vector3 mMax; ///< Upper bound of the voxel space bounding box.
    int mDiv[3];  ///< Number of divisions of the voxel space (x, y, z).

    /// Triangles.
    std::vector<Triangle> mTriangles;

    /// Vertex coordinates.
    std::vector<Vector3> mVertices;
};

}

#endif // _VOXELIZE_H_
