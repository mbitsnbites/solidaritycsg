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

#ifndef _CSGNODE_H_
#define _CSGNODE_H_

#include <list>
#include <vector>
#include "../BoundingBox.h"
#include "../Voxelize/Voxelize.h"

namespace csg {

/// CSG tree node class.
class CSGNode {
  public:
    /// Destructor.
    virtual ~CSGNode() {}

    /// Return the composite bounding box for this node (and all its children).
    virtual void SetSampleSpace(SampleSpace * aSampleSpace) = 0;

    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB) = 0;

    /// Calculate the composite slice for this node.
    virtual void ComposeSlice(Voxel * aSlice, int aZ) = 0;
};

/// Parent class for composite CSG nodes (e.g. union and intersection).
class CSGCompositeNode : public CSGNode {
  public:
    /// Destructor.
    virtual ~CSGCompositeNode();

    /// Define the sample space.
    virtual void SetSampleSpace(SampleSpace * aSampleSpace);

    /// Append a new child.
    inline CSGNode * AddChild(CSGNode * aChild)
    {
      mChildren.push_back(aChild);
      return aChild;
    }

  protected:
    std::list<CSGNode *> mChildren;  ///< List of child nodes.
    std::vector<Voxel> mTmpSlice;    ///< Intermediate slice (single child node).
};

/// Union CSG tree node class.
class CSGUnion : public CSGCompositeNode {
  public:
    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the composite slice for this node.
    virtual void ComposeSlice(Voxel * aSlice, int aZ);
};

/// Intersection CSG tree node class.
class CSGIntersection : public CSGCompositeNode {
  public:
    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the composite slice for this node.
    virtual void ComposeSlice(Voxel * aSlice, int aZ);
};

/// Difference CSG tree node class.
class CSGDifference : public CSGCompositeNode {
  public:
    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the composite slice for this node.
    virtual void ComposeSlice(Voxel * aSlice, int aZ);
};

/// Shape CSG tree node class (leaf node).
class CSGShape : public CSGNode {
  public:
    /// Constructor.
    CSGShape()
    {
      mVoxelize = 0;
    }

    /// Destructor.
    virtual ~CSGShape();

    /// Define the shape.
    Voxelize * DefineShape(Voxelize * aVoxelize);

    /// Define the sample space.
    virtual void SetSampleSpace(SampleSpace * aSampleSpace);

    /// Return the bounding box for this node.
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the slice for this node.
    virtual void ComposeSlice(Voxel * aSlice, int aZ);

  private:
    Voxelize * mVoxelize;  ///< Definition of the shape.
};

}

#endif // _CSGNODE_H_
