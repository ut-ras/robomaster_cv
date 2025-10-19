#include "Path.h"
#include <iostream>

Location::Location() : x(0), y(0), cost(0.0), costSoFar(0.0), hueristic(0.0) {}
Location::Location(int x, int y) : x(x), y(y), cost(0.0), costSoFar(0.0), hueristic(0.0) {}


bool Location::operator>(const Location& other) const
{
	return (costSoFar + hueristic) > (other.costSoFar + other.hueristic);
}

bool Location::operator==(const Location& l) const
{
	return x == l.x && y == l.y;
}

size_t LocationPtrHash::operator()(const std::shared_ptr<Location>& l) const
{
	return std::hash<int>()(l->x) ^ (std::hash<int>()(l->y) << 1);
}
bool LocationPtrEqual::operator()(const std::shared_ptr<Location>& lhs, const std::shared_ptr<Location>& rhs) const {
	return lhs->x == rhs->x && lhs->y == rhs->y;
}

Path::Path() : targetX(0), targetY(0)
{}

void Path::reloadFromGrid(Grid g)
{
	Pos targetPos = g.getTarget();
	targetX = targetPos.x;
	targetY = targetPos.y;

	m_frontier = std::priority_queue<std::shared_ptr<Location>, std::vector<std::shared_ptr<Location>>, comparator>();
	m_reached.clear();
	m_points.clear();

	//Construct set of all locations
	for (size_t x = 0; x < TILE_COUNT_X; x++)
	{
		for (size_t y = 0; y < TILE_COUNT_Y; y++)
		{
			Location tempLocation;
			tempLocation.x = (int)x;
			tempLocation.y = (int)y;
			tempLocation.hueristic = g.getHueristic((int)x, (int)y);
			tempLocation.cost = g.getCost((int)x, (int)y) + g.getAdditionalCost((int)x, (int)y);

			m_points.insert(std::make_shared<Location>(tempLocation));
		}
	}

	//Iterator over set to create paths
	for (std::shared_ptr<Location> loc : m_points)
	{
		// Create paths from each location
		std::unordered_set<std::shared_ptr<Location>,LocationPtrHash, LocationPtrEqual>::iterator it;

		if(loc->y < TILE_COUNT_Y - 1) {
			it = m_points.find(std::make_shared<Location>(loc->x, loc->y + 1));
			if(it != m_points.end() && !g.crossesWall({loc->x, loc->y}, { it->get()->x, it->get()->y })) {
				std::shared_ptr<Location> locationPointer = *it;
				loc->paths.push_back(locationPointer);
			}
		}
		if (loc->y > 0) {
			it = m_points.find(std::make_shared<Location>(loc->x, loc->y - 1));
			if (it != m_points.end() && !g.crossesWall({ loc->x, loc->y }, { it->get()->x, it->get()->y })) {
				std::shared_ptr<Location> locationPointer = *it;
				loc->paths.push_back(locationPointer);
			}
		}
		if (loc->x > 0) {
			it = m_points.find(std::make_shared<Location>(loc->x - 1, loc->y));
			if (it != m_points.end() && !g.crossesWall({ loc->x, loc->y }, { it->get()->x, it->get()->y })) {
				std::shared_ptr<Location> locationPointer = *it;
				loc->paths.push_back(locationPointer);
			}
		}
		if (loc->x < TILE_COUNT_X - 1) {
			it = m_points.find(std::make_shared<Location>(loc->x + 1, loc->y));
			if (it != m_points.end() && !g.crossesWall({ loc->x, loc->y }, { it->get()->x, it->get()->y })) {
				std::shared_ptr<Location> locationPointer = *it;
				loc->paths.push_back(locationPointer);
			}
		}
		loc.get()->paths.shrink_to_fit();
	}
}

bool Path::hasVisited(std::shared_ptr<Location> l)
{
	return m_reached.find(l) != m_reached.end();
}

std::vector<Pos> Path::calculate(Pos start)
{
	return calculate(start.x, start.y);
}

std::vector<Pos> Path::calculate(int startX, int startY)
{
	// Reset all stored values
	m_frontier = std::priority_queue<std::shared_ptr<Location>, std::vector<std::shared_ptr<Location>>, comparator>();
	m_reached.clear();
	for (std::shared_ptr<Location> l : m_points)
	{
		l->costSoFar = 0.0;
		l->from = nullptr;
	}
	
	std::unordered_set<std::shared_ptr<Location>, LocationPtrHash, LocationPtrEqual>::iterator it = m_points.find(std::make_shared<Location>(startX, startY));
	if(it == m_points.end()) {
		return std::vector<Pos>(); // Start not found
	}

	std::shared_ptr<Location> startLocation = *it;

	m_frontier.push(startLocation);

	startLocation->from = startLocation;//Starts from itself
	startLocation->costSoFar = 0.0;

	it = m_points.find(std::make_shared<Location>(targetX, targetY));
	if(it == m_points.end()) {
		return std::vector<Pos>(); // Target not found
	}
	std::shared_ptr<Location> target = *it;

	while (!m_frontier.empty())
	{
		std::shared_ptr<Location> currentLocation = m_frontier.top();
		m_frontier.pop();

		m_reached.insert(currentLocation);

		if (currentLocation == target)
		{
			// Reached the target
			std::vector<Pos> out;

			std::shared_ptr<Location> trace = currentLocation;
			while (trace != nullptr && trace != trace->from) {
				if (PRINT_TO_CONSOLE) {
					std::cout << "Path: (" << trace->x << ", " << trace->y << ")\n";
				}
				out.push_back({ trace->x, trace->y });

				trace = trace->from;
			}

			out.push_back({ startLocation->x, startLocation->y });

			return out;
		}

		// Explore neighbors
		for (std::shared_ptr<Location> neighbor : currentLocation->paths)
		{
			double newCostSoFar = currentLocation->costSoFar + neighbor->cost;

			if (neighbor != startLocation && (newCostSoFar < neighbor->costSoFar || neighbor->costSoFar == 0.0)) {
				// Found a cheaper path to the neighbor (or first time visiting)
				neighbor->costSoFar = newCostSoFar;
				neighbor->from = currentLocation;
				m_frontier.push(neighbor);
			}
		}
	}

	return std::vector<Pos>(); // Valid Path not found
	// This should probably return the most complete path in the future
}