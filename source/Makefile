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

# A simple hack to check if we are on Windows or not (i.e. are we using mingw32-make?)
ifeq ($(findstring mingw32, $(MAKE)), mingw32)
WINDOWS=1
endif

# Compiler settings
CPP = g++
CPPFLAGS = -O3 -W -Wall -c -funroll-loops
LIB = ar
LIBFLAGS = -rcs
DEPEND = g++ -MM

# Misc. system commands
ifdef WINDOWS
RM = del /Q
else
RM = rm -f
endif

# Name of the static library
STATICLIB = libsolidaritycsg.a

# Library object files
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

# Library source files
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
