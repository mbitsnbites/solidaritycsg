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
static uint32_t ReadUInt32(istream &aStream)
{
  unsigned char buf[4];
  aStream.read((char *) buf, 4);
  return ((uint32_t) buf[0]) | (((uint32_t) buf[1]) << 8) |
         (((uint32_t) buf[2]) << 16) | (((uint32_t) buf[3]) << 24);
}

/// Read a single precision 3 x float vector, endian independent.
static Vector3 ReadVector3(istream &aStream)
{
  union {
    uint32_t i;
    float  f;
  } val;
  Vector3 result;
  val.i = ReadUInt32(aStream);
  result.x = double(val.f);
  val.i = ReadUInt32(aStream);
  result.y = double(val.f);
  val.i = ReadUInt32(aStream);
  result.z = double(val.f);
  return result;
}


void STLMeshReader::LoadFromFile(const char * aFileName)
{
  // Sanity check
  if(!mMesh)
    throw runtime_error("Badly defined mesh.");

  // Open input file
  ifstream f(aFileName, ios_base::in | ios_base::binary);
  if(f.fail())
    throw runtime_error("Unable to read input file.");

  // Get the file size
  f.seekg(0, ios::end);
  uint32_t fileSize = (uint32_t) f.tellg();
  f.seekg(0, ios::beg);
  if(fileSize < 84)
    throw runtime_error("Invalid format - not a valid STL file.");

  // Read header (80 character comment + triangle count)
  f.seekg(80, ios::cur); // Skip comment
  uint32_t triangleCount = ReadUInt32(f);
  if(fileSize != (84 + triangleCount * 50))
    throw runtime_error("Not a valid STL format file.");

  if(triangleCount > 0)
  {
    // Read all the triangle data
    mMesh->mIndices.resize(triangleCount * 3);
    mMesh->mVertices.resize(triangleCount * 3);
    uint32_t idx = 0;
    for(uint32_t i = 0; i < triangleCount; ++ i)
    {
      // Skip the flat normal
      f.seekg(12, ios::cur);

      // Read the three triangle vertices
      for(uint32_t j = 0; j < 3; ++ j)
      {
        mMesh->mIndices[idx] = idx;
        mMesh->mVertices[idx] = ReadVector3(f);
        ++ idx;
      }

      // Ignore the two fill bytes
      f.seekg(2, ios::cur);
    }

    // Join all duplicate vertices
    mMesh->JoinVertices();
  }

  // Close input file
  f.close();
}

}
