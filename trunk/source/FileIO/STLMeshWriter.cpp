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
#include <cstring>
#include <stdint.h>
#include "STLMeshWriter.h"

using namespace std;

namespace csg {

/// Write a 32-bit unsigned integer, endian independent.
static void WriteUInt32(ostream &aStream, uint32_t aValue)
{
  unsigned char buf[4];
  buf[0] = aValue;
  buf[1] = aValue >> 8;
  buf[2] = aValue >> 16;
  buf[3] = aValue >> 24;
  aStream.write((char *) buf, 4);
}

/// Put a single precision float in a buffer, endian independent.
static void PutFloat(unsigned char * aBuffer, float aValue)
{
  union {
    uint32_t i;
    float  f;
  } val;

  val.f = aValue;
  aBuffer[0] = val.i;
  aBuffer[1] = val.i >> 8;
  aBuffer[2] = val.i >> 16;
  aBuffer[3] = val.i >> 24;
}


//-----------------------------------------------------------------------------
// STLMeshWriter
//-----------------------------------------------------------------------------

void STLMeshWriter::SaveToFile(const char * aFileName)
{
  // Sanity check
  if(!mMesh)
    throw runtime_error("Badly defined mesh.");

  // Open output file
  ofstream f(aFileName, ios_base::out | ios_base::binary);
  if(f.fail())
    throw runtime_error("Unable to write output file.");

  // Write header (80 character comment + triangle count)
  char comment[80];
  strncpy(comment, "Exported by SolidarityCSG", 80);
  f.write(comment, 80);
  uint32_t triangleCount = mMesh->mIndices.size() / 3;
  WriteUInt32(f, triangleCount);

  // Write all the triangle data
  int idx = 0;
  for(uint32_t i = 0; i < triangleCount; ++ i)
  {
    // Write a single triangle (normal + 3 x vertex + padding)
    char buf[50];
    unsigned char * ptr = (unsigned char *) &buf[0];

    // Get the three triangle vertices
    Vector3 * v[3];
    v[0] = &mMesh->mVertices[mMesh->mIndices[idx]];
    v[1] = &mMesh->mVertices[mMesh->mIndices[idx + 1]];
    v[2] = &mMesh->mVertices[mMesh->mIndices[idx + 2]];
    idx += 3;

    // Calulcate and put the flat triangle normal
    Vector3 n1 = *v[1] - *v[0];
    Vector3 n2 = *v[2] - *v[0];
    Vector3 n = Cross(n1, n2).Normalize();
    PutFloat(&ptr[0], float(n.x));
    PutFloat(&ptr[4], float(n.y));
    PutFloat(&ptr[8], float(n.z));
    ptr += 12;

    // Put the three triangle vertices
    for(int j = 0; j < 3; ++ j)
    {
      PutFloat(&ptr[0], float(v[j]->x));
      PutFloat(&ptr[4], float(v[j]->y));
      PutFloat(&ptr[8], float(v[j]->z));
      ptr += 12;
    }

    // Clear the two pad bytes
    ptr[0] = ptr[1] = 0;

    // Write the triangle to the output buffer
    f.write(buf, 50);
  }

  // Close output file
  f.close();
}

}
