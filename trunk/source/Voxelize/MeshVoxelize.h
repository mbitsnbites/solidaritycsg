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
#include <vector>

namespace csg {

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
    Vector3 mNormal;        ///< Triangle normal (out direction).

    friend class XYTreeNode;
    friend class ZTreeNode;
};

/// Generic binary tree node class.
class TreeNode {
  public:
    /// Generic constructor (for uninitialized nodes).
    TreeNode()
    {
      mPtr1 = mPtr2 = 0;
    }

    /// Destructor (recursively destroys all child nodes).
    ~TreeNode()
    {
      if(mPtr1 && mPtr2)
      {
        delete (TreeNode *) mPtr1;
        delete (TreeNode *) mPtr2;
      }
      else if(mPtr1)
      {
        delete (Triangle *) mPtr1;
      }
    }

    /// Check if this is a leaf node.
    inline bool IsLeafNode()
    {
      return (mPtr2 == 0);
    }

    /// Get the leaf object. If this is a parent node, return zero.
    inline Triangle * LeafItem()
    {
      if(IsLeafNode())
        return (Triangle *) mPtr1;
      else
        return (Triangle *) 0;
    }

    /// Get child A. If this is a leaf node, return zero.
    inline TreeNode * ChildA()
    {
      if(IsLeafNode())
        return (TreeNode *) 0;
      else
        return (TreeNode *) mPtr1;
    }

    /// Get child B. If this is a leaf node, return zero.
    inline TreeNode * ChildB()
    {
      return (TreeNode *) mPtr2;
    }

  protected:
    void * mPtr1;   ///< Pointer to child A (mPtr2!=0) or a Triangle object (mPtr2==0)
    void * mPtr2;   ///< Pointer to child B, or zero (for leaf nodes)
};

/// Bounding rectangle tree node class.
class XYTreeNode : public TreeNode {
  public:
    /// Constructor for leaf nodes.
    XYTreeNode(Triangle * aTriangle);

    /// Constructor for parent nodes.
    XYTreeNode(XYTreeNode * aChildA, XYTreeNode * aChildB);

    inline bool PointInside(Vector3 &aPoint)
    {
      return ((aPoint.x > mMin[0]) && (aPoint.x < mMax[0]) &&
              (aPoint.y > mMin[1]) && (aPoint.y < mMax[1]));
    }

  private:
    double mMin[2]; ///< Lower bound
    double mMax[2]; ///< Upper bound
};

/// Bounding interval tree node class.
class ZTreeNode : public TreeNode {
  public:
    /// Constructor for leaf nodes.
    ZTreeNode(Triangle * aTriangle);

    /// Constructor for parent nodes.
    ZTreeNode(ZTreeNode * aChildA, ZTreeNode * aChildB);

    inline bool PointInside(Vector3 &aPoint)
    {
      return ((aPoint.z > mMinZ) && (aPoint.z < mMaxZ));
    }

  private:
    double mMinZ;   ///< Lower bound
    double mMaxZ;   ///< Upper bound
};

/// Voxelize class for mesh objects.
class MeshVoxelize : public Voxelize {
  public:
    /// Constructor.
    MeshVoxelize()
    {
      mRectTree = 0;
      mHeightTree = 0;
    }

    /// Destructor.
    virtual ~MeshVoxelize()
    {
      if(mRectTree)
        delete mRectTree;
      if(mHeightTree)
        delete mHeightTree;
    }

    /// Define the triangle surface to be voxelized.
    /// A copy of the triangle surface is made internally in the Voxelize
    /// object, so the data arrays (aIndices, aVertices) can be free:d once this
    /// function has been called.
    void SetTriangles(int aTriangleCount, int * aIndices, int aVertexCount,
                      float * aVertices);

    /// Calculate a single slice of the voxel volume.
    /// The slice must be allocated by the caller, and hold DivX * DivY voxels.
    virtual void CalculateSlice(Voxel * aSlice, int aZ);

  private:
    /// Triangles.
    std::vector<Triangle> mTriangles;

    /// Vertex coordinates.
    std::vector<Vector3> mVertices;

    /// 2D binary rectangle tree.
    XYTreeNode * mRectTree;

    /// 1D binary interval tree.
    ZTreeNode * mHeightTree;
};

}

#endif // _MESHVOXELIZE_H_