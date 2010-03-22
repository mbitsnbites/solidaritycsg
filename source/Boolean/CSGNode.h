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

#ifndef _CSGNODE_H_
#define _CSGNODE_H_

#include <list>
#include <vector>
#include "../BoundingBox.h"
#include "../SampleSpace.h"
#include "../Voxelize/Voxelize.h"

namespace csg {

/// CSG tree node class.
class CSGNode {
  public:
    /// Destructor.
    virtual ~CSGNode() {}

    /// Define the sample space.
    virtual void SetSampleSpace(SampleSpace * aSampleSpace) = 0;

    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB) = 0;

    /// Calculate the composite slice for this node. This is just a convenience
    /// wrapper method for the ComposeSlice method.
    void CalculateSlice(Voxel * aSlice, int aZ)
    {
      int minX, minY, maxX, maxY;
      ComposeSlice(aSlice, aZ, minX, minY, maxX, maxY);
    }

    /// Calculate the composite slice for this node. The function returns false
    /// if all elements were cosidered "outside" (i.e. the slice is empty).
    /// aMinX/Y and aMaxX/Y indicate the bounding rectangle for the area of
    /// interest (output from the function). All values outside of the bounding
    /// rectangle must be marked as "outside".
    virtual bool ComposeSlice(Voxel * aSlice, int aZ, int &aMinX, int &aMinY,
                              int &aMaxX, int &aMaxY) = 0;
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
    SampleSpace * mSampleSpace;      ///< Reference to the SampleSpace object.
};

/// Union CSG tree node class.
class CSGUnion : public CSGCompositeNode {
  public:
    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the composite slice for this node. The function returns false
    /// if all elements were cosidered "outside" (i.e. the slice is empty).
    virtual bool ComposeSlice(Voxel * aSlice, int aZ, int &aMinX, int &aMinY,
                              int &aMaxX, int &aMaxY);
};

/// Intersection CSG tree node class.
class CSGIntersection : public CSGCompositeNode {
  public:
    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the composite slice for this node. The function returns false
    /// if all elements were cosidered "outside" (i.e. the slice is empty).
    virtual bool ComposeSlice(Voxel * aSlice, int aZ, int &aMinX, int &aMinY,
                              int &aMaxX, int &aMaxY);
};

/// Difference CSG tree node class.
class CSGDifference : public CSGCompositeNode {
  public:
    /// Return the composite bounding box for this node (and all its children).
    virtual void GetBoundingBox(BoundingBox &aAABB);

    /// Calculate the composite slice for this node. The function returns false
    /// if all elements were cosidered "outside" (i.e. the slice is empty).
    virtual bool ComposeSlice(Voxel * aSlice, int aZ, int &aMinX, int &aMinY,
                              int &aMaxX, int &aMaxY);
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

    /// Calculate the slice for this node. The function returns false if all
    /// elements were cosidered "outside" (i.e. the slice is empty).
    virtual bool ComposeSlice(Voxel * aSlice, int aZ, int &aMinX, int &aMinY,
                              int &aMaxX, int &aMaxY);

  private:
    Voxelize * mVoxelize;  ///< Definition of the shape.
};

}

#endif // _CSGNODE_H_
