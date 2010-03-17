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

#include <algorithm>
#include "Mesh.h"

using namespace std;

namespace csg {

/// Vertex class used when joining the triangle vertices.
class SortVertex {
  public:
    Vector3 mPoint;
    unsigned int mOldIndex;

    bool operator<(const SortVertex &v) const
    {
      return (mPoint.z < v.mPoint.z) ||
             ((mPoint.z == v.mPoint.z) && ((mPoint.y < v.mPoint.y) ||
              ((mPoint.y == v.mPoint.y) && (mPoint.x < v.mPoint.x))));
    }
};


void Mesh::JoinVertices()
{
  // Sort all vertices - O(n*log(n))
  vector<SortVertex> sortedVertices;
  sortedVertices.resize(mVertices.size());
  for(unsigned int i = 0; i < mVertices.size(); ++ i)
  {
    sortedVertices[i].mPoint = mVertices[i];
    sortedVertices[i].mOldIndex = i;
  }
  sort(sortedVertices.begin(), sortedVertices.end());

  // Join and re-index vertices - O(n)
  vector<unsigned int> indexMap;
  indexMap.resize(mVertices.size());
  SortVertex * firstEqual = &sortedVertices[0];
  int vertIdx = -1;
  for(unsigned int i = 0; i < sortedVertices.size(); ++ i)
  {
    if((i == 0) || (sortedVertices[i].mPoint != firstEqual->mPoint))
    {
      firstEqual = &sortedVertices[i];
      ++ vertIdx;
      mVertices[vertIdx] = firstEqual->mPoint;
    }
    indexMap[sortedVertices[i].mOldIndex] = vertIdx;
  }
  mVertices.resize(vertIdx + 1);

  // Re-index indices - O(n)
  for(unsigned int i = 0; i < mIndices.size(); ++ i)
    mIndices[i] = indexMap[mIndices[i]];
}

}
