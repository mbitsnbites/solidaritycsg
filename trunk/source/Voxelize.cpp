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

using namespace std;

namespace csg {

void Triangle::SetCoordinates(Vector3 * p1, Vector3 * p2, Vector3 * p3)
{
  // Assign vertex coordinate pointers
  mVertices[0] = p1;
  mVertices[1] = p2;
  mVertices[2] = p3;

  // Calculate triangle AABB
  mMin = mMax = *mVertices[0];
  for(int i = 1; i <= 2; ++ i)
  {
    if(mVertices[i]->x < mMin.x)
      mMin.x = mVertices[i]->x;
    else if(mVertices[i]->x > mMax.x)
      mMax.x = mVertices[i]->x;
    if(mVertices[i]->y < mMin.y)
      mMin.y = mVertices[i]->y;
    else if(mVertices[i]->y > mMax.y)
      mMax.y = mVertices[i]->y;
    if(mVertices[i]->z < mMin.z)
      mMin.z = mVertices[i]->z;
    else if(mVertices[i]->z > mMax.z)
      mMax.z = mVertices[i]->z;
  }

  // Calculate triangle normal (normalized cross product)
  Vector3 v1 = *p2 - *p1;
  Vector3 v2 = *p3 - *p1;
  mNormal.x = v1.y * v2.z - v1.z * v2.y;
  mNormal.y = v1.z * v2.x - v1.x * v2.z;
  mNormal.z = v1.x * v2.y - v1.y * v2.x;
  mNormal = mNormal.Normalize();
}

bool Triangle::IntersectPlane(double aPlaneZ, Vector3 * p1, Vector3 * p2)
{
  if((aPlaneZ < mMin.z) || (aPlaneZ > mMax.z))
    return false;

  double d[3], s;
  d[0] = mVertices[0]->z - aPlaneZ;
  d[1] = mVertices[1]->z - aPlaneZ;
  d[2] = mVertices[2]->z - aPlaneZ;

  if((d[0] * d[1]) < 0.0)
  {
    s = d[0] / (mVertices[1]->z - mVertices[0]->z);
    if((s < 0.0) || (s > 1.0))
      return false;
    *p1 = (*mVertices[1] - *mVertices[0]) * s + *mVertices[0];
    if((d[1] * d[2]) < 0.0)
    {
      s = d[1] / (mVertices[2]->z - mVertices[1]->z);
      if((s < 0.0) || (s > 1.0))
        return false;
      *p2 = (*mVertices[2] - *mVertices[1]) * s + *mVertices[1];
    }
    else
    {
      s = d[2] / (mVertices[0]->z - mVertices[2]->z);
      if((s < 0.0) || (s > 1.0))
        return false;
      *p2 = (*mVertices[0] - *mVertices[2]) * s + *mVertices[2];
    }
  }
  else
  {
    s = d[1] / (mVertices[2]->z - mVertices[1]->z);
    if((s < 0.0) || (s > 1.0))
      return false;
    *p1 = (*mVertices[2] - *mVertices[1]) * s + *mVertices[1];
    s = d[2] / (mVertices[0]->z - mVertices[2]->z);
    if((s < 0.0) || (s > 1.0))
      return false;
    *p2 = (*mVertices[0] - *mVertices[2]) * s + *mVertices[2];
  }

  return true;
}

double Triangle::IntersectZRay(Vector3 aOrigin)
{
  // Outside bounding rectangle?
  if((aOrigin.x < mMin.x) || (aOrigin.x > mMax.x) ||
     (aOrigin.y < mMin.y) || (aOrigin.y > mMax.y))
     return -1.0;

  // Early-out culling: is the ray origin above the triangle top?
  if(aOrigin.z > mMax.z)
     return -1.0;

  // Is the triangle parallel to the ray?
  if(fabs(mNormal.z) < 1e-50)
    return -1.0;

  Vector3 w0 = aOrigin - *mVertices[0];

  // Is the point inside the triangle (use 2D parametric coordinates)?
  // NOTE: This is a 2D version of the test in:
  //  http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm
  double u[2], v[2];
  u[0] = mVertices[1]->x - mVertices[0]->x;
  u[1] = mVertices[1]->y - mVertices[0]->y;
  v[0] = mVertices[2]->x - mVertices[0]->x;
  v[1] = mVertices[2]->y - mVertices[0]->y;
  double uu = u[0] * u[0] + u[1] * u[1];
  double uv = u[0] * v[0] + u[1] * v[1];
  double vv = v[0] * v[0] + v[1] * v[1];
  double wu = w0.x * u[0] + w0.y * u[1];
  double wv = w0.x * v[0] + w0.y * v[1];
  double D = 1.0 / (uv * uv - uu * vv);
  double s = (uv * wv - vv * wu) * D;
  if((s < 0.0) || (s > 1.0))
    return -1.0;
  double t = (uv * wu - uu * wv) * D;
  if((t < 0.0) || ((s + t) > 1.0))
    return -1.0;

  // NOTE: As an alternative method, check the winding number, see:
  //  http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm

  // Get intersection offset
  return -(mNormal.x * w0.x + mNormal.y * w0.y) / mNormal.z - w0.z;
}

Voxelize::Voxelize()
{
  // FIXME!
}

Voxelize::~Voxelize()
{
  // FIXME!
}

void Voxelize::SetTriangles(int aTriangleCount, int * aIndices,
  int aVertexCount, float * aVertices)
{
  // Create vertex array
  mVertices.resize(aVertexCount);
  float * vPtr = aVertices;
  for(int i = 0; i < aVertexCount; ++ i)
  {
    mVertices[i] = Vector3(vPtr[0], vPtr[1], vPtr[2]);
    vPtr += 3;
  }

  // Create triangle array
  mTriangles.resize(aTriangleCount);
  int * iPtr = aIndices;
  for(int i = 0; i < aVertexCount; ++ i)
  {
    mTriangles[i].SetCoordinates(&mVertices[aIndices[iPtr[0]]],
                                 &mVertices[aIndices[iPtr[1]]],
                                 &mVertices[aIndices[iPtr[2]]]);
    iPtr += 3;
  }
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

void Voxelize::CalculateSlice(Voxel * aSlice, int aZ)
{
  // FIXME!
}

}
