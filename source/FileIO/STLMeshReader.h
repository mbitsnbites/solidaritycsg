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

#ifndef _STLMESHREADER_H_
#define _STLMESHREADER_H_

#include "MeshReader.h"

namespace csg {

/// STL file format mesh reader.
class STLMeshReader : public MeshReader {
  public:
    /// Load the mesh from a file.
    virtual void LoadFromFile(const char * aFileName);
};

}

#endif // _STLMESHREADER_H_
