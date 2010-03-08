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

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <SolidarityCSG.h>

using namespace csg;
using namespace std;

int main(int argc, char ** argv)
{
  try
  {
/*
    // Test 1: Load an STL file...
    cout << "Loading STL file..." << flush;
    STLMeshReader meshReader;
    Mesh myMesh;
    meshReader.SetMesh(&myMesh);
    meshReader.LoadFromFile("bunny.stl");
    cout << "done!" << endl;

    // Test 2: ...and write it out again...
    cout << "Writing STL file..." << flush;
    STLMeshWriter meshWriter;
    meshWriter.SetMesh(&myMesh);
    meshWriter.SaveToFile("bunny-dump.stl");
    cout << "done!" << endl;
*/

    // Set up boolean tree
    cout << "Setting up CSG tree..." << flush;
    CSGDifference csg;
    CSGShape * node1 = (CSGShape *) csg.AddChild(new CSGShape());
    SphereVoxelize * s1 = (SphereVoxelize *) node1->DefineShape(new SphereVoxelize());
    s1->SetSphere(Vector3(-0.5, 0.0, 0.0), 2.0);
    CSGShape * node2 = (CSGShape *) csg.AddChild(new CSGShape());
    SphereVoxelize * s2 = (SphereVoxelize *) node2->DefineShape(new SphereVoxelize());
    s2->SetSphere(Vector3(0.5, 0.0, 0.3), 1.5);
    cout << "done!" << endl;

    // Set up a sample space
    cout << "Setting up voxel sample space..." << flush;
    SampleSpace space;
    csg.GetBoundingBox(space.mAABB);
    space.mDiv[0] = 256;
    space.mDiv[1] = 256;
    space.mDiv[2] = 256;
    csg.SetSampleSpace(&space);
    cout << "done!" << endl;

    // Prepare image writer
    TGAImageWriter output;
    output.SetFormat(space.mDiv[0], space.mDiv[1], ImageWriter::pfSigned8);

    // Begin voxelization
    cout << "Performing boolean operations..." << flush;
    vector<Voxel> voxelSlice;
    voxelSlice.resize(space.mDiv[0] * space.mDiv[1]);
    for(int i = 0; i < space.mDiv[2]; ++ i)
    {
      // Generate slice data
      csg.ComposeSlice(&voxelSlice[0], i);

      // Write this file to disk
      stringstream name;
      name << "output_";
      name.fill('0');
      name.width(5);
      name << i;
      name.width(0);
      name << ".tga";
      output.SetData(&voxelSlice[0]);
      output.SaveToFile(name.str().c_str());
    }
    cout << "done!" << endl;
  }
  catch(exception &e)
  {
    cout << "Error: " << e.what() << endl;
  }

  return 0;
}
