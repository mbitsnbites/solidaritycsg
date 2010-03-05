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
CPPFLAGS = -O3 -W -Wall -c -I../source
LFLAGS = -L../source -s
LIBS = -lsolidaritycsg
RM = del /Q

# For Windows - set this to .exe, otherwise leave it empty
EXE = .exe

all: demo$(EXE)

clean:
	$(RM) demo.o demo$(EXE)

demo$(EXE): demo.o
	$(CPP) $(LFLAGS) -o $@ $< $(LIBS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) $<
