#---------------------------------------------------------------------------------------------------
#  This file is part of SolidarityCSG.
#
#  SolidarityCSG is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  SolidarityCSG is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with SolidarityCSG.  If not, see <http://www.gnu.org/licenses/>.
#---------------------------------------------------------------------------------------------------

set(SRCS
    Voxelize/BoxVoxelize.cpp
    Voxelize/SphereVoxelize.cpp
    Voxelize/MeshVoxelize.cpp
    Voxelize/TriangleTree.cpp
    FileIO/TGAImageWriter.cpp
    FileIO/Curver.cpp
    FileIO/Mesh.cpp
    FileIO/STLMeshReader.cpp
    FileIO/STLMeshWriter.cpp
    Boolean/CSGNode.cpp
    Polygonize/Polygonize.cpp
    Polygonize/TriBuf.cpp
    SampleSpace.cpp
    BoundingBox.cpp
    Vector3.cpp
    )

add_library(solidaritycsg ${SRCS})
target_include_directories(solidaritycsg PUBLIC .)

