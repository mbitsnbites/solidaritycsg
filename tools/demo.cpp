/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <sstream>
#include <iomanip>
#include "../source/SolidarityCSG.h"

using namespace csg;
using namespace std;

int main(int argc, char ** argv)
{
  // Set up a sphere
  SphereVoxelize sv;
  sv.SetSphere(Vector3(0.0, 0.0, 0.0), 2.0);

  // Set up a sample space
  SampleSpace space;
  sv.GetBoundingBox(space.mAABB);
  space.mDiv[0] = 256;
  space.mDiv[1] = 256;
  space.mDiv[2] = 256;

  // Prepare image writer
  TGAImageWriter output;
  output.SetFormat(space.mDiv[0], space.mDiv[1], ImageWriter::pfGray8);

  // Begin voxelization
  sv.SetSampleSpace(&space);
  Voxel * voxelSlice = new Voxel[space.mDiv[0] * space.mDiv[1]];
  for(int i = 0; i < space.mDiv[2]; ++ i)
  {
    // Generate slice data
    sv.CalculateSlice(voxelSlice, i);

    // Write this file to disk
    stringstream name;
    name << "output_";
    name.fill('0');
    name.width(5);
    name << i;
    name.width(0);
    name << ".tga";
    output.SetData(voxelSlice);
    output.SaveToFile(name.str().c_str());
  }

  return 0;
}
