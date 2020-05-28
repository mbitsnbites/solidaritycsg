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

#include <stdexcept>
#include <fstream>
#include <stdint.h>
#include "STLMeshReader.h"

using namespace std;

namespace csg {

/// Read a 32-bit unsigned integer, endian independent.
static uint32_t ReadUInt32(istream& aStream) {
  unsigned char buf[4];
  aStream.read((char*)buf, 4);
  return ((uint32_t)buf[0]) | (((uint32_t)buf[1]) << 8) | (((uint32_t)buf[2]) << 16) |
         (((uint32_t)buf[3]) << 24);
}

/// Extract a single precision float from a buffer, endian independent.
static float ExtractFloat(const unsigned char* aBuffer) {
  union {
    uint32_t i;
    float f;
  } val;

  val.i = ((uint32_t)aBuffer[0]) | (((uint32_t)aBuffer[1]) << 8) | (((uint32_t)aBuffer[2]) << 16) |
          (((uint32_t)aBuffer[3]) << 24);
  return val.f;
}

//-----------------------------------------------------------------------------
// STLMeshReader
//-----------------------------------------------------------------------------

void STLMeshReader::LoadFromFile(const char* aFileName) {
  // Sanity check
  if (!mMesh)
    throw runtime_error("Badly defined mesh.");

  // Open input file
  ifstream f(aFileName, ios_base::in | ios_base::binary);
  if (f.fail())
    throw runtime_error("Unable to read input file.");

  // Get the file size
  f.seekg(0, ios::end);
  uint32_t fileSize = (uint32_t)f.tellg();
  f.seekg(0, ios::beg);
  if (fileSize < 84)
    throw runtime_error("Invalid format - not a valid STL file.");

  // Read header (80 character comment + triangle count)
  f.seekg(80, ios::cur);  // Skip comment
  uint32_t triangleCount = ReadUInt32(f);
  if (fileSize != (84 + triangleCount * 50))
    throw runtime_error("Not a valid STL format file.");

  if (triangleCount > 0) {
    // Read all the triangle data
    mMesh->mIndices.resize(triangleCount * 3);
    mMesh->mVertices.resize(triangleCount * 3);
    int idx = 0;
    for (uint32_t i = 0; i < triangleCount; ++i) {
      // Read a single triangle (normal + 3 x vertex + padding)
      char buf[50];
      f.read(buf, 50);

      // Parse the three triangle vertices
      unsigned char* ptr = (unsigned char*)&buf[12];  // Skip normal
      for (int j = 0; j < 3; ++j) {
        mMesh->mIndices[idx] = idx;
        mMesh->mVertices[idx].x = double(ExtractFloat(&ptr[0]));
        mMesh->mVertices[idx].y = double(ExtractFloat(&ptr[4]));
        mMesh->mVertices[idx].z = double(ExtractFloat(&ptr[8]));
        ptr += 12;
        ++idx;
      }
    }

    // Join all duplicate vertices
    mMesh->JoinVertices();
  }

  // Close input file
  f.close();
}

}  // namespace csg
