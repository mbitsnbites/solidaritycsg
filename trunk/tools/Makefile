CPP = g++
CPPFLAGS = -O3 -W -Wall -c

all: demo.exe

SphereVoxelize.o: ../source/Voxelize/SphereVoxelize.cpp
	$(CPP) $(CPPFLAGS) ../source/Voxelize/SphereVoxelize.cpp

TGAImageWriter.o: ../source/FileIO/TGAImageWriter.cpp
	$(CPP) $(CPPFLAGS) ../source/FileIO/TGAImageWriter.cpp

demo.o: demo.cpp
	$(CPP) $(CPPFLAGS) demo.cpp

demo.exe: demo.o SphereVoxelize.o TGAImageWriter.o
	$(CPP) demo.o SphereVoxelize.o TGAImageWriter.o -o demo.exe
