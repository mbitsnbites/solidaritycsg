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

#include "TriangleTree.h"

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// Triangle
//-----------------------------------------------------------------------------

void Triangle::SetCoordinates(Vector3 * p1, Vector3 * p2, Vector3 * p3)
{
  // Assign vertex coordinate pointers
  mVertices[0] = p1;
  mVertices[1] = p2;
  mVertices[2] = p3;

  // Calculate triangle normal (normalized cross product)
  Vector3 v1 = *p2 - *p1;
  Vector3 v2 = *p3 - *p1;
  mNormal = Cross(v1, v2).Normalize();
}

bool Triangle::IntersectPlane(double aPlaneZ, Vector3 * p1, Vector3 * p2)
{
  double d[3], s;
  d[0] = aPlaneZ - mVertices[0]->z;
  d[1] = aPlaneZ - mVertices[1]->z;
  d[2] = aPlaneZ - mVertices[2]->z;

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

double Triangle::IntersectZRay(Vector3 &aOrigin)
{
  // Is the triangle parallel to the ray?
  if(fabs(mNormal.z) < 1e-50)
    return -1.0;

  // Is the ray origin above the triangle?
  if((aOrigin.z > mVertices[0]->z) && (aOrigin.z > mVertices[1]->z) &&
     (aOrigin.z > mVertices[2]->z))
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


//-----------------------------------------------------------------------------
// AABBNode
//-----------------------------------------------------------------------------

AABBNode::AABBNode(Triangle * aTriangle)
{
  // Set pointers
  mPtr1 = (void *) aTriangle;
  mPtr2 = 0;

  // Calculate leaf node bounding box
  mAABB.mMin = mAABB.mMax = *aTriangle->mVertices[0];
  for(int i = 1; i <= 2; ++ i)
  {
    if(aTriangle->mVertices[i]->x < mAABB.mMin.x)
      mAABB.mMin.x = aTriangle->mVertices[i]->x;
    else if(aTriangle->mVertices[i]->x > mAABB.mMax.x)
      mAABB.mMax.x = aTriangle->mVertices[i]->x;
    if(aTriangle->mVertices[i]->y < mAABB.mMin.y)
      mAABB.mMin.y = aTriangle->mVertices[i]->y;
    else if(aTriangle->mVertices[i]->y > mAABB.mMax.y)
      mAABB.mMax.y = aTriangle->mVertices[i]->y;
    if(aTriangle->mVertices[i]->z < mAABB.mMin.z)
      mAABB.mMin.z = aTriangle->mVertices[i]->z;
    else if(aTriangle->mVertices[i]->z > mAABB.mMax.z)
      mAABB.mMax.z = aTriangle->mVertices[i]->z;
  }
}

AABBNode::AABBNode(AABBNode * aChildA, AABBNode * aChildB)
{
  // Set pointers
  mPtr1 = (void *) aChildA;
  mPtr2 = (void *) aChildB;

  // Calculate node bounding box
  mAABB = aChildA->mAABB;
  mAABB.Union(aChildB->mAABB);
}

void AABBNode::IntersectingTriangles(double aZ, list<Triangle *> &aList)
{
  // Is the plane outside of the bounding interval?
  if((aZ < mAABB.mMin.z) || (aZ > mAABB.mMax.z))
    return;

  Triangle * tri = LeafItem();
  if(tri)
  {
    // Add this triangle to the list
    aList.push_back(tri);
  }
  else
  {
    // Continue to recurse into the child branches
    ((AABBNode *) mPtr1)->IntersectingTriangles(aZ, aList);
    ((AABBNode *) mPtr2)->IntersectingTriangles(aZ, aList);
  }
}

int AABBNode::IntersectCount(Vector3 &aOrigin)
{
  // Is the origin outside of the bounding rectangle?
  if(((aOrigin.x < mAABB.mMin.x) || (aOrigin.x > mAABB.mMax.x) ||
      (aOrigin.y < mAABB.mMin.y) || (aOrigin.y > mAABB.mMax.y)))
    return 0;

  Triangle * tri = LeafItem();
  if(tri)
  {
    // Do we have a unique positive triangle intersection?
    double t = tri->IntersectZRay(aOrigin);
    return (t > 1e-50) ? 1 : 0;
  }
  else
  {
    // Calculate sums of the number of intersections in the child branches...
    return ((AABBNode *) mPtr1)->IntersectCount(aOrigin) +
           ((AABBNode *) mPtr2)->IntersectCount(aOrigin);
  }
}

AABBNode * BuildAABBTree(vector<AABBNode *> &aNodes, unsigned int aStart,
  unsigned int aEnd)
{
  // Leaf node?
  if(aStart == aEnd)
  {
    return aNodes[aStart];
  }

  // Calculate the combined bounding box for all the nodes in the array
  BoundingBox aabb = aNodes[aStart]->mAABB;
  for(unsigned int i = aStart + 1; i <= aEnd; ++ i)
    aabb.Union(aNodes[i]->mAABB);

  // Optimal split axis (split along X, Y or Z?)
  int axis = 0;
  if((aabb.mMax[1] - aabb.mMin[1]) > (aabb.mMax[0] - aabb.mMin[0]))
    axis = 1;
  if((aabb.mMax[2] - aabb.mMin[2]) > (aabb.mMax[axis] - aabb.mMin[axis]))
    axis = 2;

  // Partition the nodes array into the A and B branches of the new node
  double mid2 = aabb.mMin[axis] + aabb.mMax[axis];
  unsigned int storeIdx = aStart;
  AABBNode * tmp;
  for(unsigned int i = aStart; i <= aEnd; ++ i)
  {
    if((aNodes[i]->mAABB.mMin[axis] + aNodes[i]->mAABB.mMax[axis]) < mid2)
    {
      tmp = aNodes[storeIdx];
      aNodes[storeIdx] = aNodes[i];
      aNodes[i] = tmp;
      ++ storeIdx;
    }
  }

  // If we had a degenerate case, just split in half
  if((storeIdx == aStart) || (storeIdx > aEnd))
    storeIdx = (aStart + 1 + aEnd) / 2;

  // Recursively build the child branches
  AABBNode * childA = BuildAABBTree(aNodes, aStart, storeIdx - 1);
  AABBNode * childB = BuildAABBTree(aNodes, storeIdx, aEnd);

  // Create a new node, based on the A & B children
  return new AABBNode(childA, childB);
}


}
