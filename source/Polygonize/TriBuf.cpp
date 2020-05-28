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

#include "TriBuf.h"

namespace csg {

#define BUF_SIZE 65536

TriBuf::~TriBuf() {
  std::list<TriBufBin*>::iterator i;
  for (i = mBufList.begin(); i != mBufList.end(); ++i)
    delete *i;
}

void TriBuf::Append(Vector3& aPoint1, Vector3& aPoint2, Vector3& aPoint3) {
  // Will this triangle fit in the current buffer?
  if ((!mCurrent) || (mCurrent->IsFull())) {
    mCurrent = new TriBufBin(BUF_SIZE);
    mBufList.push_back(mCurrent);
  }

  // Append triangle to the current buffer bin
  Vector3* ptr = &mCurrent->mCoords[mCurrent->mCount * 3];
  ptr[0] = aPoint1;
  ptr[1] = aPoint2;
  ptr[2] = aPoint3;
  ++mCurrent->mCount;

  // Increment the total triangle count
  ++mCount;
}

void TriBuf::ToMesh(Mesh& aMesh) {
  // Copy the triangles to the destination mesh
  aMesh.mIndices.resize(mCount * 3);
  aMesh.mVertices.resize(mCount * 3);
  int* intPtr = &aMesh.mIndices[0];
  Vector3* vecPtr = &aMesh.mVertices[0];
  int idx = 0;
  std::list<TriBufBin*>::iterator i;
  for (i = mBufList.begin(); i != mBufList.end(); ++i) {
    // Copy this bin
    TriBufBin* bin = (*i);
    Vector3* srcPtr = &bin->mCoords[0];
    for (unsigned int j = 0; j < bin->mCount; ++j) {
      for (int k = 0; k < 3; ++k) {
        *intPtr = idx;
        *vecPtr = *srcPtr;
        ++intPtr;
        ++vecPtr;
        ++srcPtr;
        ++idx;
      }
    }

    // Free this bin (we're done with it)
    delete bin;
  }

  // Clear the list
  mBufList.clear();
}

}  // namespace csg
