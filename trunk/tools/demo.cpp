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

#include <iostream>
#include <vector>
#include <SolidarityCSG.h>

using namespace csg;
using namespace std;

int main()
{
  try
  {
    // Set up boolean tree
    cout << "Setting up CSG tree..." << flush;
    CSGDifference csg;
    CSGShape * node1 = (CSGShape *) csg.AddChild(new CSGShape());
    BoxVoxelize * s1 = (BoxVoxelize *) node1->DefineShape(new BoxVoxelize());
    s1->SetBox(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 1.0));
    CSGShape * node2 = (CSGShape *) csg.AddChild(new CSGShape());
    SphereVoxelize * s2 = (SphereVoxelize *) node2->DefineShape(new SphereVoxelize());
    s2->SetSphere(Vector3(0.0, 0.0, 0.0), 0.6);
    cout << "done!" << endl;

    // Set up a sample space
    cout << "Setting up voxel sample space..." << flush;
    SampleSpace space;
    BoundingBox sceneAABB;
    csg.GetBoundingBox(sceneAABB);
    space.DefineSpace(sceneAABB, 0.011);
    csg.SetSampleSpace(&space);
    cout << "done!" << endl;

    // Prepare polygonizer writer
    Polygonize polygonize;
    polygonize.SetSampleSpace(&space);

    // Begin voxelization
    cout << "Performing boolean operations..." << flush;
    vector<Voxel> voxelSlice1, voxelSlice2;
    voxelSlice1.resize(space.mDiv[0] * space.mDiv[1]);
    voxelSlice2.resize(space.mDiv[0] * space.mDiv[1]);
    Voxel * slice = &voxelSlice1[0];
    Voxel * sliceOld = &voxelSlice2[0];
    for(int i = 0; i < space.mDiv[2]; ++ i)
    {
      // Generate slice data
      csg.ComposeSlice(slice, i);

      // Convert to triangles
      if(i > 0)
        polygonize.AppendSlicePair(sliceOld, slice, i - 1);

      // Swap slice buffers
      Voxel * tmp = sliceOld;
      sliceOld = slice;
      slice = tmp;
    }
    cout << "done!" << endl;

    // Write mesh file
    cout << "Writing mesh file..." << flush;
    Mesh mesh;
    polygonize.ToMesh(mesh);
    STLMeshWriter output;
    output.SetMesh(&mesh);
    output.SaveToFile("demo.stl");
    cout << "done!" << endl;
  }
  catch(exception &e)
  {
    cout << "Error: " << e.what() << endl;
  }

  return 0;
}
