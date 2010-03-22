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

# MinGW32 4.5 linking issue...
ifdef WINDOWS
ifeq ($(findstring 4.5.,$(shell g++ -dumpversion)), 4.5.)
LFLAGS_EXTRA = -static-libstdc++ -static-libgcc
endif
endif

# Non-windwos systems need pthread
ifndef WINDOWS
LFLAGS_EXTRA = -lpthread
endif

# Main program compiler settings
CPP = g++
CPPFLAGS = -O3 -W -Wall -c -I../source -I./TinyXML -I./TinyThread $(CPPFLAGS_EXTRA)
LFLAGS = -L../source -s $(LFLAGS_EXTRA)
LIBS = -lsolidaritycsg

# TinyXML compiler settings
TINYXMLCPPFLAGS = -O2 -W -c -I./TinyXML

# TinyThread compiler settings
TINYTHREADCPPFLAGS = -O2 -W -c -I./TinyThread

# Misc. system commands
ifdef WINDOWS
RM = del /Q
else
RM = rm -f
endif

# For Windows - set this to .exe, otherwise leave it empty
ifdef WINDOWS
EXE = .exe
else
EXE =
endif

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

# TinyThread++ object files
TINYTHREADOBJS = tinythread.o

all: csg$(EXE) demo$(EXE)

clean:
	$(RM) $(CSGOBJS) $(DEMOOBJS) $(TINYXMLOBJS) $(TINYTHREADOBJS) csg$(EXE) demo$(EXE)

csg$(EXE): $(CSGOBJS) $(TINYXMLOBJS) $(TINYTHREADOBJS)
	$(CPP) $(LFLAGS) -o $@ $(CSGOBJS) $(TINYXMLOBJS) $(TINYTHREADOBJS) $(LIBS)

demo$(EXE): $(DEMOOBJS)
	$(CPP) $(LFLAGS) -o $@ $(DEMOOBJS) $(LIBS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: OS/%.cpp
	$(CPP) $(CPPFLAGS) $<

%.o: TinyXML/%.cpp
	$(CPP) $(TINYXMLCPPFLAGS) $<

%.o: TinyThread/%.cpp
	$(CPP) $(TINYTHREADCPPFLAGS) $<

# Dependencies
demo.o: demo.cpp
csg.o: csg.cpp CSGJob.h
CSGJob.o: CSGJob.cpp CSGJob.h OS/OSTime.h
OSTime.o: OS/OSTime.cpp OS/OSTime.h
