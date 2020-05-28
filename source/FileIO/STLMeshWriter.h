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

#ifndef _STLMESHWRITER_H_
#define _STLMESHWRITER_H_

#include "MeshWriter.h"

namespace csg {

/// STL file format mesh writer.
class STLMeshWriter : public MeshWriter {
public:
  /// Save the mesh to a file.
  virtual void SaveToFile(const char* aFileName);
};

}  // namespace csg

#endif  // _STLMESHWRITER_H_
