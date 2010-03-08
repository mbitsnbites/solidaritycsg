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

#ifndef _MESHREADER_H_
#define _MESHREADER_H_

#include "Mesh.h"

namespace csg {

/// Mesh reader base class.
class MeshReader {
  public:
    /// Constructor.
    MeshReader()
    {
      mMesh = 0;
    }

    /// Define the mesh data.
    inline void SetMesh(Mesh * aMesh)
    {
      mMesh = aMesh;
    }

    /// Load the mesh from a file.
    virtual void LoadFromFile(const char * aFileName) = 0;

  protected:
    Mesh * mMesh;               ///< Mesh data.
};

}

#endif // _MESHREADER_H_
