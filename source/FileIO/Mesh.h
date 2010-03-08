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

#ifndef _MESH_H_
#define _MESH_H_

#include <vector>
#include "../Vector3.h"

namespace csg {

/// 3D triangle mesh class.
class Mesh {
  public:
    /// Join duplicate (identical) vertices.
    void JoinVertices();

    std::vector<int>     mIndices;  ///< Triangle indices.
    std::vector<Vector3> mVertices; ///< Vertex coordinates.
};

}

#endif // _MESH_H_
