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

struct PointHash
{
	std::size_t operator()(Point const& p) const noexcept
	{
		std::size_t h1 = std::hash<int>{}(p.x);
		std::size_t h2 = std::hash<int>{}(p.y);
		return h1 ^ (h2 << 1); // or use boost::hash_combine (see Discussion)
	}
};

void nearestNeighborQuadTree(const vector<Node*>& inCities, vector<Node*>& outTour) {
	unordered_map<Point, vector<Node*>, PointHash> pointsToCities;
	vector<Node*> cities;
	for (int i = 0; i < inCities.size(); i++) {
		Point cityLoc = inCities[i]->loc;
		pointsToCities[cityLoc].push_back(inCities[i]);
		// Only include cities that are the first at a particular location
		// (Duplicates will be added back at the end)
		if (pointsToCities[cityLoc].size() == 1) {
			cities.push_back(inCities[i]);
		}
	}
	Quad unusedQuadTree = Quad(cities);

	vector<Node*> tour;
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

	for (int i = 0; i < tour.size(); i++) {
		outTour.push_back(tour[i]);
		Point cityLoc = tour[i]->loc;
		// Add all the cities that were filtered out
		// If none were filtered out for this location, this will be a no-op.
		for (int j = 1; j < pointsToCities[cityLoc].size(); j++) {
			outTour.push_back(pointsToCities[cityLoc][j]);
		}
	}
}


int rounded(double val) {
	return val + 0.5;
}

/****************************************************************************
**                             twoOptImprove                               **
** This function receives a tour tuple (tsp solution) and a vector graph   **
** (see 'loadMapAsGraphOfVectors' above), and attempts to restructure the  **
** the tour by 'swapping' eligible pairs of edges, if said swap reduces    **
** the total tour distance. This is in attempt to eliminate path cross-    **
** over that contributes to sub-optimality. When a swap occurs, the path   **
** in between the vertices swapped is reversed,to maintain the tour        **
** integrity. See https://en.wikipedia.org/wiki/2-opt and                  **
** http://pedrohfsd.com/2017/08/09/2opt-part1.html for more details.       **
****************************************************************************/
void twoOptImprove(vector<Node*>& tspTour)
{
	//This variable (breakOutToOptimize) is set to allow the loop to repeat
	//until the optimal improvement is obtained for small data sizes (n <= 2500).
	//In this case, execution exits both the inner and outer loops when each swap
	//is made, and the process repeats until no further improvement is possible.
	//For large data sets, the improvement runs only once (all the way through) to 
	//ensure a reasonable running time (at the expense of optimality). 
	//(See related lines 321-324 and note the control statement 
	//`breakOutToOptimize == false` in both the inner and outer for loops.)
	bool breakOutToOptimize;
	bool nExceeds2500;
	bool improved;
	if (tspTour.size() > 2500)
	{
		nExceeds2500 = true;
	}
	else
	{
		nExceeds2500 = false;
	}

	do
	{
		improved = false;
		breakOutToOptimize = false;
		//(Can't swap 1st city so i starts at 1...)
		for (int i = 1; i < tspTour.size() - 2 &&
			breakOutToOptimize == false; i++)
		{
			for (int j = i, k = i + 1; k < tspTour.size() &&
				breakOutToOptimize == false; k++)
			{
				//Adjacent vertices are not eligible for consideration
				//because there is only one edge between them.
				if (k - j == 1)
				{
					continue;
				}

				//If distance(i to i + j -1) + distance(i + 1, j) <
				//   distance(i to i + 1) + distance(j - 1 to j), the swap
				//will improve the tour. In other words, if taking out the two
				//edges before the swap and inserting two new edges (because of swap)
				//results in shorter tour, the cities are swapped in tour order.
				if (rounded(tspTour[j]->loc.distTo(tspTour[k - 1]->loc)) +
					rounded(tspTour[j + 1]->loc.distTo(tspTour[k]->loc)) <
					rounded(tspTour[j]->loc.distTo(tspTour[j + 1]->loc)) +
					rounded(tspTour[k - 1]->loc.distTo(tspTour[k]->loc)))
				{

					improved = true;
					//Only need to reverse cities in between swapped routes (edges).
					for (int l = j + 1, m = k - 1; l < m; l++, m--)
					{
						Node* temp = tspTour[l];
						tspTour[l] = tspTour[m];
						tspTour[m] = temp;
					}
					//Allow optimization of improvement if data size is manageable.
					//(Otherwise, additional time cost is unreasonable, run 2Opt-swap once over only.)
					//If enabled (i.e. data set <= 1000), execution exits both inner and outer
					//loop after improvement to start over.
					if (!nExceeds2500)
					{
						breakOutToOptimize = true;
					}
				}
			}
		}
	} while (improved);
}

int getTourLength(const vector<Node*>& tour) {
	int length = 0;
	for (int i = 0; i < tour.size(); i++) {
		int compareIdx = i + 1;
		if (compareIdx == tour.size()) {
			compareIdx = 0;
		}
		Point p1 = tour[i]->loc;
		Point p2 = tour[compareIdx]->loc;
		length += rounded(p1.distTo(p2));
	}
	return length;
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
	cout << "tour construction: " << duration << " seconds" << endl;

	high_resolution_clock::time_point t3 = high_resolution_clock::now();
	twoOptImprove(tour);
	high_resolution_clock::time_point t4 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(t4 - t3).count() / 1000000.0f;
	cout << "tour optimization: " << duration2 << " seconds" << endl;

	auto durationTotal = duration_cast<microseconds>(t4 - t1).count() / 1000000.0f;
	cout << "total time: " << durationTotal << " seconds" << endl;

	cout << "tour length: " << getTourLength(tour) << endl;

	printTour(tour, inFilename);

	return 0;
}

