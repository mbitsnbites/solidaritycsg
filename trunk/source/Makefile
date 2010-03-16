#-----------------------------------------------------------------------------------------
# This file is part of SolidarityCSG.
#
# SolidarityCSG is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SolidarityCSG is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with SolidarityCSG.  If not, see <http://www.gnu.org/licenses/>.
#-----------------------------------------------------------------------------------------

CPP = g++
CPPFLAGS = -O3 -W -Wall -c
LIB = ar
LIBFLAGS = -rcs
DEPEND = g++ -MM
#RM = del /Q
RM = rm -f

STATICLIB = libsolidaritycsg.a

OBJS = BoxVoxelize.o \
       SphereVoxelize.o \
       MeshVoxelize.o \
       TGAImageWriter.o \
       Mesh.o \
       STLMeshReader.o \
       STLMeshWriter.o \
       CSGNode.o \
       Polygonize.o \
       TriBuf.o \
       SampleSpace.o \
       BoundingBox.o \
       Vector3.o

SRCS = Voxelize/BoxVoxelize.cpp \
       Voxelize/SphereVoxelize.cpp \
       Voxelize/MeshVoxelize.cpp \
       FileIO/TGAImageWriter.cpp \
       FileIO/Mesh.cpp \
       FileIO/STLMeshReader.cpp \
       FileIO/STLMeshWriter.cpp \
       Boolean/CSGNode.cpp \
       Polygonize/Polygonize.cpp \
       Polygonize/TriBuf.cpp \
       SampleSpace.cpp \
       BoundingBox.cpp \
       Vector3.cpp

all: $(STATICLIB)

clean:
	$(RM) $(OBJS) $(STATICLIB)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: Voxelize/%.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: FileIO/%.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: Boolean/%.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: Polygonize/%.cpp
	$(CPP) $(CPPFLAGS) $<

$(STATICLIB): $(OBJS)
	$(LIB) $(LIBFLAGS) $@ $(OBJS)

depend:
	$(DEPEND) $(SRCS) > make.depend

-include make.depend
