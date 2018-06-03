
all: QuadTree RunTSP Group65TSP

QuadTree: QuadTree.cpp
	g++ -O2 -std=c++11 -o QuadTree.o -c QuadTree.cpp

RunTSP: RunTSP.cpp
	g++ -O2 -std=c++11 -o RunTSP.o -c RunTSP.cpp

Group65TSP: QuadTree.o RunTSP.o
	g++ -O2 -std=c++11 -o Group65TSP QuadTree.o RunTSP.o

clean:
	rm -rf QuadTree.o RunTSP.o Group65TSP
