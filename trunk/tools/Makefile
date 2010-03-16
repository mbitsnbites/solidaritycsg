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

# Main program compiler settings
CPP = g++
CPPFLAGS = -O3 -W -Wall -c -I../source -I./TinyXML
LFLAGS = -L../source -s
LIBS = -lsolidaritycsg

# TinyXML compiler settings
TINYCPPFLAGS = -O2 -W -c -I./TinyXML

# Misc. system commands
#RM = del /Q
RM = rm -f

# For Windows - set this to .exe, otherwise leave it empty
#EXE = .exe
EXE =

# CSG program object files
CSGOBJS = csg.o \
          CSGJob.o \
          OSTime.o

# Demo program object files
DEMOOBJS = demo.o

# TinyXML object files
TINYXMLOBJS = tinyxml.o \
              tinyxmlerror.o \
              tinyxmlparser.o \
              tinystr.o

all: csg$(EXE) demo$(EXE)

clean:
	$(RM) $(CSGOBJS) $(DEMOOBJS) $(TINYXMLOBJS) csg$(EXE) demo$(EXE)

csg$(EXE): $(CSGOBJS) $(TINYXMLOBJS)
	$(CPP) $(LFLAGS) -o $@ $(CSGOBJS) $(TINYXMLOBJS) $(LIBS)

demo$(EXE): $(DEMOOBJS)
	$(CPP) $(LFLAGS) -o $@ $(DEMOOBJS) $(LIBS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: OS/%.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: TinyXML/%.cpp
	$(CPP) $(TINYCPPFLAGS) $<

# Dependencies
demo.o: demo.cpp
csg.o: csg.cpp CSGJob.h
CSGJob.o: CSGJob.cpp CSGJob.h OS/OSTime.h
OSTime.o: OS/OSTime.cpp OS/OSTime.h
