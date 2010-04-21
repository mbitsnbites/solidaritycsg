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

#ifndef _TRIANGLETREE_H_
#define _TRIANGLETREE_H_

#include "../Vector3.h"
#include "../BoundingBox.h"
#include <vector>
#include <list>

namespace csg {

// Forward declarations
class Mesh;


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
    double IntersectZRay(Vector3 &aOrigin);

  private:
    Vector3 * mVertices[3]; ///< Pointers to the actual vertex coordinates.
    Vector3 mNormal;        ///< Triangle normal (out direction).

    friend class AABBNode;
};


/// Generic binary tree node class.
class TreeNode {
  public:
    /// Generic constructor (for uninitialized nodes).
    TreeNode() : mPtr1(0), mPtr2(0) {}

    /// Destructor (recursively destroys all child nodes).
    ~TreeNode()
    {
      if(mPtr1 && mPtr2)
      {
        delete (TreeNode *) mPtr1;
        delete (TreeNode *) mPtr2;
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

/// Axis aligned bounding box tree node class.
class AABBNode : public TreeNode {
  public:
    /// Constructor for leaf nodes.
    AABBNode(Triangle * aTriangle);

    /// Constructor for parent nodes.
    AABBNode(AABBNode * aChildA, AABBNode * aChildB);

    /// Generate a list of all triangles that intersect the given XY plane.
    void IntersectingTriangles(double aZ, std::list<Triangle *> &aList);

    /// Check if a point is inside or outside of the triangle volume. Note
    /// that this method only makes sense for the root node of the tree.
    inline bool PointInside(Vector3 &aPoint)
    {
      return (IntersectCount(aPoint) & 1) ? true : false;
    }

  private:
    /// Calculate number of intersections with a positive Z ray starting at
    /// aPoint.
    int IntersectCount(Vector3 &aOrigin);

    BoundingBox mAABB; ///< Bounding box for this node

    friend AABBNode * BuildAABBTree(std::vector<AABBNode *> &aNodes, unsigned int aStart, unsigned int aEnd);
};


/// Build an AABB tree from a list of nodes.
AABBNode * BuildAABBTree(std::vector<AABBNode *> &aNodes, unsigned int aStart, unsigned int aEnd);


}

#endif // _TRIANGLETREE_H_
