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

#ifndef _MESHWRITER_H_
#define _MESHWRITER_H_

#include "Mesh.h"

namespace csg {

/// Mesh writer base class.
class MeshWriter {
  public:
    /// Constructor.
    MeshWriter()
    {
      mMesh = 0;
    }

    /// Define the mesh data.
    inline void SetMesh(Mesh * aMesh)
    {
      mMesh = aMesh;
    }

    /// Save the mesh to a file.
    virtual void SaveToFile(const char * aFileName) = 0;

  protected:
    Mesh * mMesh;               ///< Mesh data.
};

}

#endif // _MESHWRITER_H_
