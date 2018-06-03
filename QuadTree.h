/*
Copyright 2018 Mindy Jones
*/

#pragma once

#include <cstdint>
#include <vector>

enum NeighborCode { 
	NC_RIGHT,
	NC_LEFT,
	NC_DOWN,
	NC_UP,
	NC_RIGHT_UP,
	NC_RIGHT_DOWN,
	NC_LEFT_DOWN,
	NC_LEFT_UP,
	NC_HALT
};

enum QuadNum { TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT };

int QT_addressLen(uint64_t address);
uint64_t QT_makeChildAddress(uint64_t parent, int quadNum);
bool QT_isXPrefixOfY(uint64_t x, uint64_t y);
int QT_getQuadNum(uint64_t address, int index);
uint64_t QT_replaceQuadNum(uint64_t address, int index, int newQuadNum);
uint64_t QT_getNeighborAddress(uint64_t start, NeighborCode firstCode);

struct Point
{
	int x;
	int y;
	Point();
	Point(int x, int y);
	bool operator==(const Point& rhs) const;
	uint64_t distSquaredTo(const Point& other);
	double distTo(const Point& other);
};

struct Node
{
	Point loc;
	int val;
	Node(Point loc, int val);
};

// The main quadtree class
class Quad
{
	Point topLeft;
	Point bottomRight;

	Quad *parent;
	uint64_t address;

	Node *node;
	Quad *children[4];

public:
	Quad(const std::vector<Node*>& initialNodes);
	Quad(Point inTopLeft, Point inBottomRight);

	// returns true if inserted, otherwise false
	bool insert(Node* inNode);

	// returns true if removed, otherwise false. Caller must delete `node`.
	bool remove(Node* inNode);

	// returns true if this quad has no nodes
	bool isEmpty();

	// returns the nearest neighbor, including p itself if a Node is located at p
	Node* getNearestNeighbor(Point p);

	// returns the nearest neighbor, excluding the point p if a Node is located at p
	Node* getNearestNeighborNoExactMatch(Point p);

	// get the lowest quad in the tree matching the target address
	Quad* getQuad(uint64_t targetAddress);

	// get the lowest quad in the tree containing the given point
	Quad* findSmallestContainingQuad(Point);

	// returns true if the given point is in this quad
	bool isInQuad(Point);

	// returns the address of this quad
	uint64_t getAddress();

	// computes the number of nodes by navigating the entire tree to count them - O(n)
	int getSizeSlow();

	void getNearestNodeInside(Point p, uint64_t& closestDistSquared, Node*& closestNode);

private:
	Quad(Quad *parent, Node *inNode, uint64_t address, Point inTopLeft, Point inBottomRight);
	bool insertImpl(Node* inNode);
	Node* getNearestNeighbor(Point p, bool excludeExactMatch);
};
