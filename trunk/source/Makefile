#-----------------------------------------------------------------------------------------
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#-----------------------------------------------------------------------------------------

CPP = g++
CPPFLAGS = -g -O3 -W -Wall -c
LIB = ar
LIBFLAGS = -rcs
DEPEND = g++ -MM
RM = del /Q

STATICLIB = libsolidaritycsg.a

OBJS = SphereVoxelize.o \
       MeshVoxelize.o \
       TGAImageWriter.o \
       CSGNode.o \
       BoundingBox.o

SRCS = Voxelize/SphereVoxelize.cpp \
       Voxelize/MeshVoxelize.cpp \
       FileIO/TGAImageWriter.cpp \
       Boolean/CSGNode.cpp \
       BoundingBox.cpp

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

$(STATICLIB): $(OBJS)
	$(LIB) $(LIBFLAGS) $@ $(OBJS)

depend:
	$(DEPEND) $(SRCS) > make.depend

-include make.depend
