/*
Copyright 2018 Mindy Jones, Ben Fridkis, Mathew Kagel
Project for CS 325 (Algorithms), Oregon State University
*/

#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <tuple>
#include <queue>

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

// Based on http://tsp-basics.blogspot.com/2017/03/shifting-segment.html
void shiftSegment(vector<Node*>& tour, int i, int j, int k) {
	int segmentSize = (j - i + tour.size()) % tour.size();
	int shiftSize = ((k - i + tour.size()) - segmentSize + tour.size()) % tour.size();
	int offset = i + 1 + shiftSize;
	vector<Node*> segment;
	for (int pos = 0; pos < segmentSize; pos++) {
		segment.push_back(tour[(pos + i + 1) % tour.size()]);
	}
	int pos = (i + 1) % tour.size();
	for (int counter = 1; counter <= shiftSize; counter++) {
		tour[pos] = tour[(pos + segmentSize) % tour.size()];
		pos = (pos + 1) % tour.size();
	}
	for (pos = 0; pos < segmentSize; pos++) {
		tour[(pos + offset) % tour.size()] = segment[pos];
	}
}

/*
Based on http://tsp-basics.blogspot.com/2017/03/or-opt.html
Assumes: X1!=Z1
         X2==successor(X1); Y2==successor(Y1); Z2==successor(Z1)
*/
int gainFromSegmentShift(Point x1, Point x2, Point y1, Point y2, Point z1, Point z2) {
	int deleteLength = x1.distTo(x2) + y1.distTo(y2) + z1.distTo(z2);
	int addLength = x1.distTo(y2) + z1.distTo(x2) + y1.distTo(z2);
	return deleteLength - addLength;
}

/*
Based on http://tsp-basics.blogspot.com/2017/03/or-opt.html
*/
void performOrOpt(vector<Node*>& tour, high_resolution_clock::time_point programStart) {
	bool locallyOptimal = false;
	int i;
	int j;
	int k;
	int optRuns = 0;
	high_resolution_clock::time_point start = high_resolution_clock::now();
	high_resolution_clock::time_point lastPrint = high_resolution_clock::now();

	int searchWindow = 0;
	int maxSearch = 1000;
	while (searchWindow < maxSearch &&
		duration_cast<microseconds>(high_resolution_clock::now() - programStart).count() / 1000000.0f < 178.0) {
		if (locallyOptimal) {
			searchWindow += 10;
		}

		//cout << "---" << endl;
		//cout << "after or-opt run " << optRuns << ": " << getTourLength(tour) << endl;
		optRuns++;
		high_resolution_clock::time_point now = high_resolution_clock::now();
		
		auto lastPrintDuration = duration_cast<microseconds>(now - lastPrint).count() / 1000000.0f;
		if (lastPrintDuration > 10) {
			auto duration = duration_cast<microseconds>(now - start).count() / 1000000.0f;
			cout << "  time: " << duration << " seconds" << endl;
			lastPrint = high_resolution_clock::now();
		}

		locallyOptimal = true;
		for (int segmentLen = 3; segmentLen >= 1; segmentLen--) {
			bool break2 = false;
			for (int pos = 0; pos < tour.size() && !break2; pos++) {
				int i = pos;
				Point x1 = tour[i]->loc;
				Point x2 = tour[(i + 1) % tour.size()]->loc;
				int j = (i + segmentLen) % tour.size();
				Point y1 = tour[j]->loc;
				Point y2 = tour[(j + 1) % tour.size()]->loc;
				
				int shiftCount = 0;
				int posShift = segmentLen + 1;
				int negShift = tour.size() - 1;
				while (posShift < searchWindow && negShift > tour.size() - searchWindow) {
					int shift;
					if (shiftCount % 2 == 0) {
						shift = posShift;
						posShift++;
					}
					else {
						shift = negShift;
						negShift--;
					}
					shiftCount++;

					int k = (i + shift) % tour.size();
					Point z1 = tour[k]->loc;
					Point z2 = tour[(k + 1) % tour.size()]->loc;

					if (gainFromSegmentShift(x1, x2, y1, y2, z1, z2) > 0) {
						shiftSegment(tour, i, j, k);
						//cout << "i = " << i << ", j = " << j << ", k = " << k << "; shift = " << shift << endl;
						locallyOptimal = false;
						break2 = true;
						break;
					}
				}
			}

		}
	}
}

//Structure to represent an edge. Each CityDistance structure
//hold the distance to a particular city.
struct CityDistance {
	int city;
	int nextCity;
	int distanceToCity;
	CityDistance() {};
	CityDistance(int c, int nc, int dTC)
	{
		city = c;
		nextCity = nc;
		distanceToCity = dTC;
	}
};

//Used to modify the stl priority_heap from max heap to min heap.
//Reference: https://www.geeksforgeeks.org/implement-min-heap-using-stl/
class myComparator
{
public:
	int operator() (const CityDistance& c1, const CityDistance& c2)
	{
		return c1.distanceToCity > c2.distanceToCity;
	}
};

typedef priority_queue<CityDistance, vector<CityDistance>, myComparator> CityDistancePQ;

/**************************************************************************************
**                          loadGraphOfMapAsPriorityQueue                            **
** This function creates the map representation in memory. It returns a min-heap 	 **
** holding all edges of the graph, with the edge that has the minimum distance as	 **
** the "root" of the heap.                                                           **
**************************************************************************************/
void loadCityPriorityQueue(const vector<Node*>& cities, CityDistancePQ& graph)
{
	for (int i = 0; i < cities.size(); i++) {
		for (int j = 0; j < cities.size(); j++) {
			int dist = rounded(cities[i]->loc.distTo(cities[j]->loc));
			graph.push(CityDistance(cities[i]->val, cities[j]->val, dist));
		}
	}
}

void buildGreedyTour(const vector<Node*>& cities, CityDistancePQ& graph, vector<Node*>& tspTour)
{
	int cityCount = cities.size();

	//The vector of tuples below (cityTourPositionTracker)
	//stores for each city a boolean to indicate if the city
	//has been added to the tour, an integer to indicate
	//the city's forward adjacent city (or "next city"),
	//and an integer to indicate the city's backward adjacent
	//city (or "previous city). A value of -1 for the next city
	//item (2nd tuple item) or previous city item (3rd tuple item)
	//indicates that a city has not been assigned a next city or
	//previous city, respectively.
	vector<tuple<bool, int, int>> cityTourPositionTracker;
	for (int i = 0; i < cityCount; i++)
	{
		cityTourPositionTracker.push_back(make_tuple(false, -1, -1));
	}

	int distance = 0;
	for (int i = 0; i < cityCount; i++)
	{
		bool edgeAdded = false;
		while (!edgeAdded)
		{
			//The while loop below removes any ineligible edge (except for those
			//that result in a cycle (which are handled further down). Ineligible
			//edges removed here are those with cities (vertices) that already have
			//two adjacent cities (vertices) and self-referential edges.
			while (get<0>(cityTourPositionTracker[graph.top().city]) == true ||
				get<2>(cityTourPositionTracker[graph.top().nextCity]) != -1 ||
				graph.top().city == graph.top().nextCity)
			{
				graph.pop();
			}
			//Make sure the current edge
			//will not create a cycle (except when adding the final
			//edge) when added to the tour. If adding the edge will
			//create a cycle, it is discarded and the compatibility
			//checks restart in the next iteration fo the while loop.
			int downstreamCity = graph.top().nextCity;
			bool edgeCreatesCycle = false;

			//(Note the last edge added is allowed to make the cycle, hence the condition
			//"i < cityCount - 1" in the while loop below.)
			while (get<0>(cityTourPositionTracker[downstreamCity]) == true &&
				get<1>(cityTourPositionTracker[downstreamCity]) != -1 && i < cityCount - 1)
			{
				downstreamCity = get<1>(cityTourPositionTracker[downstreamCity]);
				if (downstreamCity == graph.top().city)
				{
					graph.pop();
					edgeCreatesCycle = true;
					break;
				}
			}

			if (!edgeCreatesCycle)
			{
				get<0>(cityTourPositionTracker[graph.top().city]) = true;
				get<1>(cityTourPositionTracker[graph.top().city]) = graph.top().nextCity;
				get<2>(cityTourPositionTracker[graph.top().nextCity]) = graph.top().city;
				distance += graph.top().distanceToCity;
				graph.pop();
				edgeAdded = true;
			}
		}
	}
	for (int i = 0, j = 0; i < cityCount; i++)
	{
		tspTour.push_back(cities[j]);
		j = get<1>(cityTourPositionTracker[j]);
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

	high_resolution_clock::time_point programStart = high_resolution_clock::now();
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
	
	//CityDistancePQ graph;
	//loadCityPriorityQueue(cities, graph);
	//buildGreedyTour(cities, graph, tour);

	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(t2 - t1).count() / 1000000.0f;
	cout << "tour construction: " << duration << " seconds" << endl;
	cout << "tour length A: " << getTourLength(tour) << endl;

	high_resolution_clock::time_point t3 = high_resolution_clock::now();
	twoOptImprove(tour);
	high_resolution_clock::time_point t4 = high_resolution_clock::now();
	auto duration2 = duration_cast<microseconds>(t4 - t3).count() / 1000000.0f;
	cout << "tour optimization 1 time: " << duration2 << " seconds" << endl;
	cout << "tour length B: " << getTourLength(tour) << endl;

	high_resolution_clock::time_point t5 = high_resolution_clock::now();
	performOrOpt(tour, programStart);
	high_resolution_clock::time_point t6 = high_resolution_clock::now();
	auto duration3 = duration_cast<microseconds>(t6 - t5).count() / 1000000.0f;
	cout << "tour optimization 2 time: " << duration3 << " seconds" << endl;
	cout << "tour length C: " << getTourLength(tour) << endl;

	auto durationTotal = duration_cast<microseconds>(t6 - t1).count() / 1000000.0f;
	cout << "total time: " << durationTotal << " seconds" << endl;

	printTour(tour, inFilename);

	return 0;
}


