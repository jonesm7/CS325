/*
Copyright 2018 Mindy Jones
*/

#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <unordered_map>

#include "QuadTree.h"

using namespace std;
using namespace std::chrono;

void nearestNeighborQuadTree(const vector<Node*>& cities, vector<Node*>& tour) {
	Quad unusedQuadTree = Quad(cities);

	tour.push_back(cities[0]);
	unusedQuadTree.remove(cities[0]);

	Node* lastNode = cities[0];
	int i = 0;
	while (!unusedQuadTree.isEmpty()) {
		Node* closestNode = unusedQuadTree.getNearestNeighbor(lastNode->loc);
		if (closestNode == NULL) {
			cout << "Hit NULL node, exiting" << endl;
			return;
		}
		tour.push_back(closestNode);
		unusedQuadTree.remove(closestNode);
		lastNode = closestNode;
		i++;
	}
}

void printTour(const vector<Node*>& tour, string outputFileBase) {
	int length = 0;
	for (int i = 0; i < tour.size(); i++) {
		int compareIdx = i + 1;
		if (compareIdx == tour.size()) {
			compareIdx = 0;
		}
		Point p1 = tour[i]->loc;
		Point p2 = tour[compareIdx]->loc;
		length += p1.distTo(p2) + 0.5;
	}

	string outFileName = outputFileBase + ".tour";
	ofstream outfile(outFileName);
	if (!outfile.is_open()) {
		cerr << "Couldn't write to file: " << outFileName.c_str() << endl;
		return;
	}
	outfile << length << endl;
	for (int i = 0; i < tour.size(); i++) {
		outfile << tour[i]->val << endl;
	}
	outfile.close();
}

int main(int argc, char * argv[])
{
	if (argc != 2) {
		cout << "Wrong number of arguments - expected 1" << endl;
		return 1;
	}
	string inFilename = argv[1];
	
	ifstream infile(inFilename);
	if (!infile.is_open()) {
		cerr << "Couldn't open file: " << inFilename.c_str() << endl;
		return 1;
	}
	cout << "Processing " << inFilename.c_str() << endl;

	vector<Node*> cities;

	while (infile) {
		int val;
		int x;
		int y;

		if (!(infile >> val)) {
			break;
		}
		infile >> x;
		infile >> y;

		Node* node = new Node(Point(x, y), val);
		cities.push_back(node);
	}

	cout << cities.size() << " cities" << endl;

	vector<Node*> tour;
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	nearestNeighborQuadTree(cities, tour);
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(t2 - t1).count() / 1000000.0f;
	cout << "duration: " << duration << " seconds" << endl;

	printTour(tour, inFilename);

	return 0;
}

