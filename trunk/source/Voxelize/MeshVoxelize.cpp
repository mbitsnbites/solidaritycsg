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
#include <stdexcept>

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
// XYTreeNode
//-----------------------------------------------------------------------------

XYTreeNode::XYTreeNode(Triangle * aTriangle)
{
  // Set pointers
  mPtr1 = (void *) aTriangle;
  mPtr2 = 0;

  // Calculate leaf node bounding rectangle
  mMin[0] = mMax[0] = aTriangle->mVertices[0]->x;
  mMin[1] = mMax[1] = aTriangle->mVertices[0]->y;
  for(int i = 1; i <= 2; ++ i)
  {
    if(aTriangle->mVertices[i]->x < mMin[0])
      mMin[0] = aTriangle->mVertices[i]->x;
    else if(aTriangle->mVertices[i]->x > mMax[0])
      mMax[0] = aTriangle->mVertices[i]->x;
    if(aTriangle->mVertices[i]->y < mMin[1])
      mMin[1] = aTriangle->mVertices[i]->y;
    else if(aTriangle->mVertices[i]->y > mMax[1])
      mMax[1] = aTriangle->mVertices[i]->y;
  }
}

XYTreeNode::XYTreeNode(XYTreeNode * aChildA, XYTreeNode * aChildB)
{
  // Set pointers
  mPtr1 = (void *) aChildA;
  mPtr2 = (void *) aChildB;

  // Calculate node bounding rectangle
  for(int i = 0; i < 2; ++ i)
  {
    if(aChildA->mMin[i] < aChildB->mMin[i])
      mMin[i] = aChildA->mMin[i];
    else
      mMin[i] = aChildB->mMin[i];
    if(aChildA->mMax[i] > aChildB->mMax[i])
      mMax[i] = aChildA->mMax[i];
    else
      mMax[i] = aChildB->mMax[i];
  }
}

int XYTreeNode::IntersectCount(Vector3 &aOrigin)
{
  // Is the origin outside of the bounding rectangle?
  if(!((aOrigin.x > mMin[0]) && (aOrigin.x < mMax[0]) &&
       (aOrigin.y > mMin[1]) && (aOrigin.y < mMax[1])))
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
    return ((XYTreeNode *) mPtr1)->IntersectCount(aOrigin) +
           ((XYTreeNode *) mPtr2)->IntersectCount(aOrigin);
  }
}

//-----------------------------------------------------------------------------
// ZTreeNode
//-----------------------------------------------------------------------------

ZTreeNode::ZTreeNode(Triangle * aTriangle)
{
  // Set pointers
  mPtr1 = (void *) aTriangle;
  mPtr2 = 0;

  // Calculate leaf node bounding interval
  mMinZ = mMaxZ = aTriangle->mVertices[0]->z;
  for(int i = 1; i <= 2; ++ i)
  {
    if(aTriangle->mVertices[i]->z < mMinZ)
      mMinZ = aTriangle->mVertices[i]->z;
    else if(aTriangle->mVertices[i]->z > mMaxZ)
      mMaxZ = aTriangle->mVertices[i]->z;
  }
}

ZTreeNode::ZTreeNode(ZTreeNode * aChildA, ZTreeNode * aChildB)
{
  // Set pointers
  mPtr1 = (void *) aChildA;
  mPtr2 = (void *) aChildB;

  // Calculate node bounding interval
  if(aChildA->mMinZ < aChildB->mMinZ)
    mMinZ = aChildA->mMinZ;
  else
    mMinZ = aChildB->mMinZ;
  if(aChildA->mMaxZ > aChildB->mMaxZ)
    mMaxZ = aChildA->mMaxZ;
  else
    mMaxZ = aChildB->mMaxZ;
}

void ZTreeNode::IntersectingTriangles(double aZ, list<Triangle *> &aList)
{
  // Is the plane outside of the bounding interval?
  if((aZ < mMinZ) || (aZ > mMaxZ))
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
    ((ZTreeNode *) mPtr1)->IntersectingTriangles(aZ, aList);
    ((ZTreeNode *) mPtr2)->IntersectingTriangles(aZ, aList);
  }
}

//-----------------------------------------------------------------------------
// MeshVoxelize
//-----------------------------------------------------------------------------

void MeshVoxelize::SetTriangles(Mesh &aMesh)
{
  SetTriangles(int(aMesh.mIndices.size() / 3), &aMesh.mIndices[0],
               int(aMesh.mVertices.size()), &aMesh.mVertices[0].x);
}

void MeshVoxelize::SetTriangles(int aTriangleCount, int * aIndices,
  int aVertexCount, double * aVertices)
{
  // Sanity check
  if((aTriangleCount < 4) || (!aIndices) || (aVertexCount < 4) || (!aVertices))
    throw runtime_error("Invalid mesh definition.");

  // Create vertex array, and calculate the shape bounding box
  mVertices.resize(aVertexCount);
  double * vPtr = aVertices;
  mAABB.mMin = mAABB.mMax = Vector3(vPtr[0], vPtr[1], vPtr[2]);
  for(int i = 0; i < aVertexCount; ++ i)
  {
    // Copy vertex coordinates
    mVertices[i] = Vector3(vPtr[0], vPtr[1], vPtr[2]);

    // Update bounding box min/max
    if(mVertices[i].x < mAABB.mMin.x)
      mAABB.mMin.x = mVertices[i].x;
    else if(mVertices[i].x > mAABB.mMax.x)
      mAABB.mMax.x = mVertices[i].x;
    if(mVertices[i].y < mAABB.mMin.y)
      mAABB.mMin.y = mVertices[i].y;
    else if(mVertices[i].y > mAABB.mMax.y)
      mAABB.mMax.y = mVertices[i].y;
    if(mVertices[i].z < mAABB.mMin.z)
      mAABB.mMin.z = mVertices[i].z;
    else if(mVertices[i].z > mAABB.mMax.z)
      mAABB.mMax.z = mVertices[i].z;

    vPtr += 3;
  }

  // Create triangle array
  mTriangles.resize(aTriangleCount);
  int * iPtr = aIndices;
  for(int i = 0; i < aTriangleCount; ++ i)
  {
    mTriangles[i].SetCoordinates(&mVertices[iPtr[0]],
                                 &mVertices[iPtr[1]],
                                 &mVertices[iPtr[2]]);
    iPtr += 3;
  }

  // Build the 2D bounding rectangle tree (XY)
  vector<XYTreeNode *> rectLeafNodes(mTriangles.size(), 0);
  try
  {
    for(unsigned int i = 0; i < mTriangles.size(); ++ i)
      rectLeafNodes[i] = new XYTreeNode(&mTriangles[i]);
    mRectTree = BuildRectangleTree(rectLeafNodes);
  }
  catch(...)
  {
    // If something went wrong, free all the leaf nodes
    for(unsigned int i = 0; i < rectLeafNodes.size(); ++ i)
      if(rectLeafNodes[i])
        delete rectLeafNodes[i];
    throw;
  }
  rectLeafNodes.clear();

  // Build the 1D bounding interval tree (Z)
  vector<ZTreeNode *> heightLeafNodes(mTriangles.size(), 0);
  try
  {
    for(unsigned int i = 0; i < mTriangles.size(); ++ i)
      heightLeafNodes[i] = new ZTreeNode(&mTriangles[i]);
    mHeightTree = BuildHeightTree(heightLeafNodes);
  }
  catch(...)
  {
    // If something went wrong, free all the leaf nodes
    for(unsigned int i = 0; i < heightLeafNodes.size(); ++ i)
      if(heightLeafNodes[i])
        delete heightLeafNodes[i];
    throw;
  }
  heightLeafNodes.clear();
}

void MeshVoxelize::CalculateSlice(Voxel * aSlice, int aZ)
{
  // Check that the voxel space has been properly set up
  if(!mSampleSpace || !mSampleSpace->IsValid())
    throw runtime_error("Undefined/invalid voxel space dimensions.");

  // Check that the mesh has been properly set up
  if(!mRectTree || !mHeightTree)
    throw runtime_error("Undefined triangle mesh.");

  // Calculate the slice plane Z value
  Vector3 voxelSize = mSampleSpace->VoxelSize();
  double planeZ = aZ * voxelSize.z + mSampleSpace->mAABB.mMin.z;

  // Start by marking all voxels of the slice as "UNVISITED"
  Voxel * ptr = aSlice;
  for(int y = 0; y < mSampleSpace->mDiv[1]; ++ y)
    for(int x = 0; x < mSampleSpace->mDiv[0]; ++ x)
      *ptr ++ = VOXEL_UNVISITED;

  // Get all intersecting triangles
  list<Triangle *> triList;
  mHeightTree->IntersectingTriangles(planeZ, triList);

  // Calculate and draw all intersections to the slice
  for(list<Triangle *>::iterator t = triList.begin(); t != triList.end(); ++ t)
  {
    Vector3 p1, p2;
    if((*t)->IntersectPlane(planeZ, &p1, &p2))
      DrawLineSegment(aSlice, p1, p2);
  }

  // Flood fill the unvisited parts of the slice according to in/out
  ptr = aSlice;
  for(int y = 0; y < mSampleSpace->mDiv[1]; ++ y)
  {
    for(int x = 0; x < mSampleSpace->mDiv[0]; ++ x)
    {
      if(*ptr == VOXEL_UNVISITED)
      {
        // Calculate voxel 3D coordinate
        Vector3 p;
        p.x = x * voxelSize.x + mSampleSpace->mAABB.mMin.x;
        p.y = y * voxelSize.y + mSampleSpace->mAABB.mMin.y;
        p.z = planeZ;

        // Check if the voxel is inside or outside of the triangle mesh
        Voxel value = mRectTree->PointInside(p) ? VOXEL_MAX : -VOXEL_MAX;

        // Flood fill from this voxel
        FloodFill(aSlice, x, y, value);
      }
    }
    ptr ++;
  }
}

XYTreeNode * MeshVoxelize::BuildRectangleTree(vector<XYTreeNode *> &aNodes)
{
  // Leaf node?
  if(aNodes.size() == 1)
    return aNodes[0];

  // Calculate the combined bounding rectangle for all the nodes in the array
  double min[2], max[2];
  min[0] = aNodes[0]->mMin[0];
  min[1] = aNodes[0]->mMin[1];
  max[0] = aNodes[0]->mMax[0];
  max[1] = aNodes[0]->mMax[1];
  for(unsigned int i = 1; i < aNodes.size(); ++ i)
  {
    for(int j = 0; j < 2; ++ j)
    {
      if(aNodes[i]->mMin[j] < min[j])
        min[j] = aNodes[i]->mMin[j];
      if(aNodes[i]->mMax[j] > max[j])
        max[j] = aNodes[i]->mMax[j];
    }
  }

  // Optimal split axis (split along X or along Y?)
  int axis = ((max[1] - min[1]) > (max[0] - min[0])) ? 1 : 0;

  // Partition the nodes array into the A and B branches of the new node
  vector<XYTreeNode *> childANodes(aNodes.size());
  vector<XYTreeNode *> childBNodes(aNodes.size());
  double mid2 = min[axis] + max[axis];
  int countA = 0;
  int countB = 0;
  for(unsigned int i = 0; i < aNodes.size(); ++ i)
  {
    if((aNodes[i]->mMin[axis] + aNodes[i]->mMax[axis]) < mid2)
    {
      childANodes[countA] = aNodes[i];
      ++ countA;
    }
    else
    {
      childBNodes[countB] = aNodes[i];
      ++ countB;
    }
  }

  // If we had a degenerate case, move one node from the full branch to the
  // empty branch
  if(countA == 0)
  {
    childANodes[0] = childBNodes[countB - 1];
    ++ countA;
    -- countB;
  }
  else if(countB == 0)
  {
    childBNodes[0] = childANodes[countA - 1];
    -- countA;
    ++ countB;
  }

  // Adjust the sizes of the arrays
  childANodes.resize(countA);
  childBNodes.resize(countB);

  // Recursively build the child branches
  XYTreeNode * childA = BuildRectangleTree(childANodes);
  childANodes.clear();
  XYTreeNode * childB = BuildRectangleTree(childBNodes);
  childBNodes.clear();

  // Create a new node, based on the A & B children
  return new XYTreeNode(childA, childB);
}

ZTreeNode * MeshVoxelize::BuildHeightTree(vector<ZTreeNode *> &aNodes)
{
  // Leaf node?
  if(aNodes.size() == 1)
    return aNodes[0];

  // Calculate the combined bounding interval for all the nodes in the array
  double minZ, maxZ;
  minZ = aNodes[0]->mMinZ;
  maxZ = aNodes[0]->mMaxZ;
  for(unsigned int i = 1; i < aNodes.size(); ++ i)
  {
    if(aNodes[i]->mMinZ < minZ)
      minZ = aNodes[i]->mMinZ;
    if(aNodes[i]->mMaxZ > maxZ)
      maxZ = aNodes[i]->mMaxZ;
  }

  // Partition the nodes array into the A and B branches of the new node
  vector<ZTreeNode *> childANodes(aNodes.size());
  vector<ZTreeNode *> childBNodes(aNodes.size());
  double mid2 = minZ + maxZ;
  int countA = 0;
  int countB = 0;
  for(unsigned int i = 0; i < aNodes.size(); ++ i)
  {
    if((aNodes[i]->mMinZ + aNodes[i]->mMaxZ) < mid2)
    {
      childANodes[countA] = aNodes[i];
      ++ countA;
    }
    else
    {
      childBNodes[countB] = aNodes[i];
      ++ countB;
    }
  }

  // If we had a degenerate case, move one node from the full branch to the
  // empty branch
  if(countA == 0)
  {
    childANodes[0] = childBNodes[countB - 1];
    ++ countA;
    -- countB;
  }
  else if(countB == 0)
  {
    childBNodes[0] = childANodes[countA - 1];
    -- countA;
    ++ countB;
  }

  // Adjust the sizes of the arrays
  childANodes.resize(countA);
  childBNodes.resize(countB);

  // Recursively build the child branches
  ZTreeNode * childA = BuildHeightTree(childANodes);
  childANodes.clear();
  ZTreeNode * childB = BuildHeightTree(childBNodes);
  childBNodes.clear();

  // Create a new node, based on the A & B children
  return new ZTreeNode(childA, childB);
}

void MeshVoxelize::DrawLineSegment(Voxel * aSlice, Vector3 &p1, Vector3 &p2)
{
  // FIXME
}

void MeshVoxelize::FloodFill(Voxel * aSlice, int aX, int aY, Voxel aValue)
{
  // Sanity check
  if((aValue == VOXEL_UNVISITED) || (aX < 0) || (aX >= mSampleSpace->mDiv[0]) ||
     (aY < 0) || (aY >= mSampleSpace->mDiv[1]))
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
  while(!queue.empty())
  {
    // Get next voxel to evaluate
    int idx = queue.front();
    queue.pop_front();

    // Convert 1D index to the corresponding x coordinate
    int x = idx % xCount;

    // Find first non-filled voxel along this column
    while((idx >= 0) && (aSlice[idx] == VOXEL_UNVISITED))
      idx -= xCount;
    idx += xCount;

    // Fill this column as far as its unvisited
    bool spanLeft = false;
    bool spanRight = false;
    while((idx < sliceSize) && (aSlice[idx] == VOXEL_UNVISITED))
    {
      // Set fill value
      aSlice[idx] = aValue;

      // Append neighbours to the fill queue
      if((!spanLeft) && (x > 0) && (aSlice[idx - 1] == VOXEL_UNVISITED))
      {
        queue.push_back(idx - 1);
        spanLeft = true;
      }
      else if(spanLeft && (x > 0) && (aSlice[idx - 1] != VOXEL_UNVISITED))
      {
        spanLeft = false;
      }
      if((!spanRight) && (x < xMax) && (aSlice[idx + 1] == VOXEL_UNVISITED))
      {
        queue.push_back(idx + 1);
        spanRight = true;
      }
      else if(spanRight && (x < xMax) && (aSlice[idx + 1] != VOXEL_UNVISITED))
      {
        spanRight = false;
      }

      idx += xCount;
    }
  }
}

}