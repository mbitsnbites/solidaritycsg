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

#ifndef _CURVER_H_
#define _CURVER_H_

#include "../SampleSpace.h"
#include <vector>
#include <list>

namespace csg {

/// 2D point class.
class Point {
  public:
    Point() : x(0), y(0) {}

    Point(double aX, double aY)
    {
      x = aX;
      y = aY;
    }

    inline double& operator[](unsigned int aIdx) const
    {
      return ((double *) &x)[aIdx];
    }

    inline Point operator+(const Point &p) const
    {
      return Point(x + p.x,  y + p.y);
    }

    inline Point operator-(const Point &p) const
    {
      return Point(x - p.x,  y - p.y);
    }

    inline void operator+=(const Point &p)
    {
      x += p.x;
      y += p.y;
    }

    inline bool operator==(const Point &p) const
    {
      return (x == p.x) && (y == p.y);
    }

    inline bool operator!=(const Point &p) const
    {
      return (x != p.x) || (y != p.y);
    }

    double x, y;
};


/// Closed 2D countour class.
class Contour {
  public:
    /// Check if the countour is an exterior.
    /// Any contour that is counter clockwise oriented is considered exterior.
    bool IsExterior();

    /// Return the number of points in the contour.
    inline unsigned int Count()
    {
      return mPoints.size();
    }

    inline Point& operator[](unsigned int aIdx)
    {
      return mPoints[aIdx];
    }

  private:
    std::vector<Point> mPoints;
};

/// Bitmap to curve converter.
/// This class implements the marching squares algorithm, and generates
/// separated closed contours from a voxel slice.
class Curver {
  public:
    /// Constructor.
    Curver() : mSampleSpace(0) {}

    /// Destructor.
    ~Curver();

    /// Define the sample space (dimensions).
    inline void SetSampleSpace(SampleSpace * aSampleSpace)
    {
      mSampleSpace = aSampleSpace;
    }

    /// Make contours from a slice.
    /// @note The sample space must have been defined first (see SetSampleSpace).
    void MakeContours(Voxel * aSlice);

  private:
    std::list<Contour*> mContours; ///< List of contours.
    SampleSpace * mSampleSpace;    ///< Sample space definition
};


}

#endif // _CURVER_H_
