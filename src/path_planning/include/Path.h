#pragma once

#include <queue>
#include <memory>

#include "Grid.h"
#include <set>
#include <unordered_set>

struct Location {
	int x;
	int y;

	double cost;
	double hueristic;
	double costSoFar;

	std::vector<std::shared_ptr<Location>> paths;
	std::shared_ptr<Location> from{ nullptr };

	Location();
	Location(int x, int y);

	bool operator>(const Location& other) const;
	bool operator==(const Location& l) const;
};

struct LocationPtrHash {
	size_t operator()(const std::shared_ptr<Location>& l) const;
};

struct LocationPtrEqual {
	bool operator()(const std::shared_ptr<Location>& lhs, const std::shared_ptr<Location>& rhs) const;
};

struct comparator
{
	bool operator()(std::shared_ptr<Location> lhs, std::shared_ptr<Location> rhs) const
	{
		if(lhs == nullptr || rhs == nullptr) {
			return false;
		}
		return *lhs.get() > *rhs.get();
	}
};

class Path
{
public:
	Path();

	void reloadFromGrid(Grid g);

	bool hasVisited(std::shared_ptr<Location>);

	std::vector<Pos> calculate(Pos start);
	std::vector<Pos> calculate(int startX, int startY);

	std::unordered_set<std::shared_ptr<Location>, LocationPtrHash, LocationPtrEqual> m_points;//Swap for better type if needed
private:

	int targetX;
	int targetY;

	std::priority_queue<std::shared_ptr<Location>, std::vector<std::shared_ptr<Location>>, comparator> m_frontier;
	std::unordered_set<std::shared_ptr<Location>> m_reached;

};

