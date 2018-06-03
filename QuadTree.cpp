/*
Copyright 2018 Mindy Jones
This implementation uses logic from the paper "A Practical Algorithm for
Computing Neighbors in Quadtrees, Octrees, and Hyperoctrees" by Yoder, Bloniarz 2006.
*/
#include "QuadTree.h"

#include <cmath>
#include <cstddef>

using namespace std;

const int LEN_BITS = 8;

int QT_addressLen(uint64_t address) {
	return 0xff & address;
}

uint64_t QT_makeChildAddress(uint64_t parent, int quadNum) {
	int length = QT_addressLen(parent);
	uint64_t dataPart = parent >> LEN_BITS;
	uint64_t withNewDim = (dataPart << 2) | (quadNum & 0x3);
	uint64_t withNewLen = (withNewDim << LEN_BITS) | ((length + 1) & 0xff);
	return withNewLen;
}

bool QT_isXPrefixOfY(uint64_t x, uint64_t y) {
	int xLen = QT_addressLen(x);
	int yLen = QT_addressLen(y);
	if (xLen > yLen) {
		return false;
	}
	uint64_t xAddressData = x >> LEN_BITS;
	uint64_t yAddressData = y >> LEN_BITS;
	int shiftDifference = (yLen - xLen) * 2;
	return (yAddressData >> shiftDifference) == xAddressData;
}

int QT_getQuadNum(uint64_t address, int index) {
	int len = QT_addressLen(address);
	uint64_t addressComponent = address >> LEN_BITS;
	int bitShift = (len - index - 1) * 2;
	uint64_t shifted = addressComponent >> bitShift;
	return shifted & 3;
}

uint64_t QT_replaceQuadNum(uint64_t address, int index, int newQuadNum) {
	int len = QT_addressLen(address);
	int bitShift = (len - index - 1) * 2;
	uint64_t zeroMask = ~(3 << (bitShift + LEN_BITS));
	uint64_t numShifted = (newQuadNum & 0x3) << (bitShift + LEN_BITS);
	uint64_t newAddress = (address & zeroMask) | numShifted;
	return newAddress;
}

const int DIRECTIONS = 8;

int FSM_QUAD[][4] = {
	{1, 0, 3, 2},
	{1, 0, 3, 2},
	{2, 3, 0, 1},
	{2, 3, 0, 1},
	{3, 2, 1, 0},
	{3, 2, 1, 0},
	{3, 2, 1, 0},
	{3, 2, 1, 0},
};

NeighborCode FSM_CODE[][4] = {
	{ NC_HALT, NC_RIGHT, NC_HALT, NC_RIGHT },
	{ NC_LEFT, NC_HALT, NC_LEFT, NC_HALT },
	{ NC_HALT, NC_HALT, NC_DOWN, NC_DOWN },
	{ NC_UP, NC_UP, NC_HALT, NC_HALT },
	{ NC_UP, NC_RIGHT_UP, NC_HALT, NC_RIGHT },
	{ NC_HALT, NC_RIGHT, NC_DOWN, NC_RIGHT_DOWN },
	{ NC_LEFT, NC_HALT, NC_LEFT_DOWN, NC_DOWN },
	{ NC_LEFT_UP, NC_UP, NC_LEFT, NC_HALT }
};

uint64_t QT_getNeighborAddress(uint64_t start, NeighborCode firstCode) {
	uint64_t newAddress = start;
	int len = QT_addressLen(start);
	NeighborCode curCode = firstCode;
	for (int i = len - 1; i >= 0; i--) {
		int quadNum = QT_getQuadNum(start, i);
		int newQuadNum = FSM_QUAD[curCode][quadNum];
		NeighborCode newCode = FSM_CODE[curCode][quadNum];
		curCode = newCode;
		newAddress = QT_replaceQuadNum(newAddress, i, newQuadNum);
		if (curCode == NC_HALT) {
			return newAddress;
		}
	}
	return -1;
}

Point::Point()
{
	x = 0;
	y = 0;
}

Point::Point(int inX, int inY)
{
	x = inX;
	y = inY;
}

bool Point::operator==(const Point& rhs) const
{
	return x == rhs.x && y == rhs.y;
}

uint64_t Point::distSquaredTo(const Point& other)
{
	int xdiff = x - other.x;
	int ydiff = y - other.y;
	return xdiff*xdiff + ydiff*ydiff;
}

double Point::distTo(const Point& other)
{
	int xdiff = x - other.x;
	int ydiff = y - other.y;
	return sqrt(xdiff*xdiff + ydiff*ydiff);
}

Node::Node(Point inLoc, int inVal): loc(inLoc)
{
	val = inVal;
}

const int NUM_QUADS = 4;

Quad::Quad(const vector<Node*>& initialNodes)
{
	int minX = INT32_MAX;
	int maxX = INT32_MIN;
	int minY = INT32_MAX;
	int maxY = INT32_MIN;

	for (int i = 0; i < initialNodes.size(); i++) {
		if (initialNodes[i]->loc.x < minX) {
			minX = initialNodes[i]->loc.x;
		}
		if (initialNodes[i]->loc.x > maxX) {
			maxX = initialNodes[i]->loc.x;
		}
		if (initialNodes[i]->loc.y < minY) {
			minY = initialNodes[i]->loc.y;
		}
		if (initialNodes[i]->loc.y > maxY) {
			maxY = initialNodes[i]->loc.y;
		}
	}

	topLeft = Point(minX, minY);
	bottomRight = Point(maxX, maxY);
	parent = NULL;
	address = 0;
	node = NULL;

	if (initialNodes.size() == 0) {
		return;
	}

	node = initialNodes[0];
	for (int i = 0; i < NUM_QUADS; i++) {
		children[i] = NULL;
	}

	for (int i = 1; i < initialNodes.size(); i++) {
		insertImpl(initialNodes[i]);
	}
}

Quad::Quad(Point inTopLeft, Point inBottomRight)
{
	topLeft = inTopLeft;
	bottomRight = inBottomRight;
	parent = NULL;
	address = 0;

	node = NULL;
	for (int i = 0; i < NUM_QUADS; i++) {
		children[i] = NULL;
	}
}

Quad::Quad(Quad *par, Node *inNode, uint64_t addr, Point inTopLeft, Point inBottomRight)
{
	topLeft = inTopLeft;
	bottomRight = inBottomRight;
	parent = par;
	address = addr;

	node = inNode;
	for (int i = 0; i < NUM_QUADS; i++) {
		children[i] = NULL;
	}
}

// Insert a node into the quadtree
bool Quad::insert(Node *inNode)
{
	if (isEmpty()) {
		node = inNode;
		return true;
	}
	else {
		return insertImpl(inNode);
	}
}

bool Quad::insertImpl(Node *inNode)
{
	if (inNode == NULL) {
		return false;
	}

	// Current quad cannot contain it
	if (!isInQuad(inNode->loc)) {
		return false;
	}

	// This was a Quad of 1, now we need to split it apart - insert existing
	//   node under a new Quad
	if (node != NULL) {
		Node *existingNode = node;
		node = NULL;
		if (!insertImpl(existingNode)) {
			return false;
		}
	}

	if ((topLeft.x + bottomRight.x) / 2 >= inNode->loc.x) {
		// top left
		if ((topLeft.y + bottomRight.y) / 2 >= inNode->loc.y) {
			if (children[TOP_LEFT] == NULL) {
				children[TOP_LEFT] = new Quad(
					this,
					inNode,
					QT_makeChildAddress(address, TOP_LEFT),
					Point(topLeft.x, topLeft.y),
					Point((topLeft.x + bottomRight.x) / 2,
						(topLeft.y + bottomRight.y) / 2));
			}
			else {
				children[TOP_LEFT]->insertImpl(inNode);
			}
		}
		// bottom left
		else
		{
			if (children[BOT_LEFT] == NULL) {
				children[BOT_LEFT] = new Quad(
					this,
					inNode,
					QT_makeChildAddress(address, BOT_LEFT),
					Point(topLeft.x,
						(topLeft.y + bottomRight.y) / 2),
					Point((topLeft.x + bottomRight.x) / 2,
						bottomRight.y));
			}
			else {
				children[BOT_LEFT]->insertImpl(inNode);
			}
		}
	}
	else
	{
		// top right
		if ((topLeft.y + bottomRight.y) / 2 >= inNode->loc.y) {
			if (children[TOP_RIGHT] == NULL) {
				children[TOP_RIGHT] = new Quad(
					this,
					inNode,
					QT_makeChildAddress(address, TOP_RIGHT),
					Point((topLeft.x + bottomRight.x) / 2,
						topLeft.y),
					Point(bottomRight.x,
						(topLeft.y + bottomRight.y) / 2));
			}
			else {
				children[TOP_RIGHT]->insertImpl(inNode);
			}
		}
		// bottom right
		else
		{
			if (children[BOT_RIGHT] == NULL) {
				children[BOT_RIGHT] = new Quad(
					this,
					inNode,
					QT_makeChildAddress(address, BOT_RIGHT),
					Point((topLeft.x + bottomRight.x) / 2,
						(topLeft.y + bottomRight.y) / 2),
					Point(bottomRight.x, bottomRight.y));
			}
			else {
				children[BOT_RIGHT]->insertImpl(inNode);
			}
		}
	}
	return true;
}

bool Quad::remove(Node* inNode)
{
	Quad* curQuad = findSmallestContainingQuad(inNode->loc);
	if (curQuad->node != inNode) {
		return false;
	}
	curQuad->node = NULL;
	if (this == curQuad) {
		return true;
	}

	// count siblings, and NULL-out the child pointer of the removed node
	Quad* parentQuad = curQuad->parent;
	int siblingCount = 0;
	int lastSiblingIdx = -1;
	Quad* lastSibling = NULL;
	for (int i = 0; i < NUM_QUADS; i++) {
		if (parentQuad->children[i] == curQuad) {
			parentQuad->children[i] = NULL;
		}
		else if (parentQuad->children[i] != NULL) {
			siblingCount++;
			lastSibling = parentQuad->children[i];
			lastSiblingIdx = i;
		}
	}
	delete curQuad;
	// if there is more than one other node in this quadrant, stop.
	// siblingCount should never be 0 here
	if (siblingCount > 1 || (siblingCount == 1 && lastSibling->node == NULL)) {
		return true;
	}

	// the last sibling only has 1 node, so promote it to the parent quad
	parentQuad->children[lastSiblingIdx] = NULL;
	parentQuad->node = lastSibling->node;
	delete lastSibling;

	// Move the node up until there are siblings or we reach the root
	curQuad = parentQuad;
	while (curQuad->parent != NULL) {
		Quad* parentQuad = curQuad->parent;
		int curQuadIdx = -1;
		siblingCount = 0;
		for (int i = 0; i < NUM_QUADS; i++) {
			if (parentQuad->children[i] == curQuad) {
				curQuadIdx = i;
			}
			else if (parentQuad->children[i] != NULL) {
				siblingCount++;
			}
		}
		if (siblingCount == 0) {
			parentQuad->node = curQuad->node;
			parentQuad->children[curQuadIdx] = NULL;
			delete curQuad;
		}
		else {
			return true;
		}
		curQuad = parentQuad;
	}
	return true;
}

bool Quad::isEmpty()
{
	if (node != NULL) {
		return false;
	}
	else {
		// count up children
		int childrenCount = 0;
		for (int i = 0; i < NUM_QUADS; i++) {
			if (children[i] != NULL) {
				childrenCount++;
			}
		}
		if (childrenCount == 0) {
			return true;
		}
		else {
			return false;
		}
	}

}

Node* Quad::getNearestNeighbor(Point p)
{
	return getNearestNeighbor(p, false);
}

Node* Quad::getNearestNeighborNoExactMatch(Point p)
{
	return getNearestNeighbor(p, true);
}

Node* Quad::getNearestNeighbor(Point p, bool excludeExactMatch)
{
	// Current quad cannot contain it
	if (!isInQuad(p)) {
		return NULL;
	}

	Quad* containingQuad = findSmallestContainingQuad(p);
	if (!containingQuad) {
		return NULL;
	}

	uint64_t closestDistSquared = UINT64_MAX;
	Node* closestNode = NULL;

	if (containingQuad->node != NULL) {
		if (containingQuad->node->loc.x == p.x && containingQuad->node->loc.y == p.y) {
			if (!excludeExactMatch) {
				return containingQuad->node;
			}
		}
		else {
			closestDistSquared = p.distSquaredTo(containingQuad->node->loc);
			closestNode = containingQuad->node;
		}
	}
	else {
		containingQuad->getNearestNodeInside(p, closestDistSquared, closestNode);
	}

	for (int dir = 0; dir < DIRECTIONS; dir++) {
		uint64_t neighborAddress =
			QT_getNeighborAddress(containingQuad->address, (NeighborCode) dir);
		Quad* neighborQuad = containingQuad->getQuad(neighborAddress);
		if (neighborQuad == NULL) {
			continue;
		}
		if (QT_isXPrefixOfY(neighborQuad->address, containingQuad->address)) {
			// the lowest quad containing the neighbor also contains this quad,
			// so searching it is redundant
			continue;
		}
		neighborQuad->getNearestNodeInside(p, closestDistSquared, closestNode);
	}

	// Check and see if the distance from the input point to the closest point
	// found in the surrounding 8 quads is longer than the distance from
	// the input point to any of the edges of the 8 quads. If so, we need to check
	// in the quads surrounding the adjacent 8.
	int quadSize = containingQuad->bottomRight.x - containingQuad->topLeft.x;
	float closestDist = sqrt(closestDistSquared);
	bool expand1 = false;
	expand1 |= (p.x - containingQuad->topLeft.x) + quadSize < closestDist;
	expand1 |= (containingQuad->bottomRight.x - p.x) + quadSize < closestDist;
	expand1 |= (p.y - containingQuad->topLeft.y) + quadSize < closestDist;
	expand1 |= (containingQuad->bottomRight.y - p.y) + quadSize < closestDist;

	if (expand1) {
		Quad* neighborQuad = containingQuad;
		uint64_t nextAddress = QT_getNeighborAddress(containingQuad->address, NC_LEFT_UP);
		neighborQuad = neighborQuad->getQuad(nextAddress);
		nextAddress = QT_getNeighborAddress(nextAddress, NC_LEFT_UP);
		neighborQuad = neighborQuad->getQuad(nextAddress);

		NeighborCode sequence[] = {
			NC_RIGHT, NC_RIGHT, NC_RIGHT, NC_RIGHT,
			NC_DOWN, NC_DOWN, NC_DOWN, NC_DOWN,
			NC_LEFT, NC_LEFT, NC_LEFT, NC_LEFT,
			NC_UP, NC_UP, NC_UP, NC_UP
		};
		for (int i = 0; i < 16; i++) {
			nextAddress = QT_getNeighborAddress(nextAddress, sequence[i]);
			neighborQuad = neighborQuad->getQuad(nextAddress);
			if (neighborQuad == NULL) {
				continue;
			}
			if (QT_isXPrefixOfY(neighborQuad->address, containingQuad->address)) {
				// the lowest quad containing the neighbor also contains this quad,
				// so searching it is redundant
				continue;
			}
			neighborQuad->getNearestNodeInside(p, closestDistSquared, closestNode);
		}
	}

	return closestNode;
}

Quad* Quad::getQuad(uint64_t targetAddress) {
	Quad* curQuad = this;

	// traverse up the tree to the common ancestor
	while (!QT_isXPrefixOfY(curQuad->address, targetAddress)) {
		curQuad = curQuad->parent;
		if (curQuad == NULL) {
			return NULL;
		}
	}

	// traverse down the tree to the lowest containing quad
	while (curQuad->address != targetAddress
		&& QT_addressLen(curQuad->address) < QT_addressLen(targetAddress))
	{
		// no child quads, so stop here
		if (curQuad->node != NULL) {
			break;
		}
		int nextQuadNumIndex = QT_addressLen(curQuad->address);
		int nextQuadNum = QT_getQuadNum(targetAddress, nextQuadNumIndex);
		if (curQuad->children[nextQuadNum] != NULL) {
			curQuad = curQuad->children[nextQuadNum];
		}
		else {
			// There are no nodes in the area of the desired quad
			break;
		}
	}

	return curQuad;
}

// Search all points inside this Quad for nearest node
void Quad::getNearestNodeInside(Point p, uint64_t& closestDistSquared, Node*& closestNode) {
	if (node != NULL) {
		uint64_t thisDistSquared = p.distSquaredTo(node->loc);
		if (closestNode == NULL || thisDistSquared < closestDistSquared) {
			closestNode = node;
			closestDistSquared = thisDistSquared;
		}
	}
	else {
		for (int i = 0; i < NUM_QUADS; i++) {
			if (children[i] != NULL) {
				children[i]->getNearestNodeInside(p, closestDistSquared, closestNode);
			}
		}
	}
}

Quad* Quad::findSmallestContainingQuad(Point p)
{
	if (!isInQuad(p)) {
		return NULL;
	}

	// We are at a quad containing only one node
	if (node != NULL) {
		return this;
	}

	if ((topLeft.x + bottomRight.x) / 2 >= p.x) {
		// top left
		if ((topLeft.y + bottomRight.y) / 2 >= p.y) {
			if (children[TOP_LEFT] == NULL) {
				return this;
			}
			else {
				return children[TOP_LEFT]->findSmallestContainingQuad(p);
			}
		}
		// bottom right
		else {
			if (children[BOT_LEFT] == NULL) {
				return this;
			}
			else {
				return children[BOT_LEFT]->findSmallestContainingQuad(p);
			}
		}
	}
	else
	{
		// top right
		if ((topLeft.y + bottomRight.y) / 2 >= p.y) {
			if (children[TOP_RIGHT] == NULL) {
				return this;
			}
			else {
				return children[TOP_RIGHT]->findSmallestContainingQuad(p);
			}
		}
		// bottom right
		else {
			if (children[BOT_RIGHT] == NULL) {
				return this;
			}
			else {
				return children[BOT_RIGHT]->findSmallestContainingQuad(p);
			}
		}
	}
};

// Check if current quadtree contains the point
bool Quad::isInQuad(Point p)
{
	return (p.x >= topLeft.x &&
		p.x <= bottomRight.x &&
		p.y >= topLeft.y &&
		p.y <= bottomRight.y);
}

uint64_t Quad::getAddress() {
	return address;
}

int Quad::getSizeSlow() {
	if (node != NULL) {
		return 1;
	}
	else {
		int childrenSizes = 0;
		for (int i = 0; i < NUM_QUADS; i++) {
			if (children[i] != NULL) {
				childrenSizes += children[i]->getSizeSlow();
			}
		}
		return childrenSizes;
	}
}
