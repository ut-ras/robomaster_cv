#pragma once
#include "AStarConfig.h"
#include <array>
#include <cmath>


// The Grid class should act as an abstraction of the gridded field.
// Should handle finding the hueristic of a specific tile as well as the cost of traversing tiles.

struct Pos
{
	int x;
	int y;
};

class Grid
{
public:
	Grid();

	// Getters and setters for target
	void setTarget(int x, int y);
	void setTarget(Pos target);
	Pos getTarget();

	// Getters and setters for hueristic
	double getHueristic(int x, int y) const;
	void setHueristic(int x, int y, double hueristic);

	// Getters and setters for cost
	double getCost(int x, int y) const;
	void setCost(int x, int y, double cost);

	// Getters and setters for additional cost map
	double getAdditionalCost(int x, int y) const;
	void setAdditionalCost(int x, int y, double cost);

	// Static helper methods to determine if a tile is an edge tile or rough terrain
	static bool isRoughTerrain(Pos p);
	static bool isEdgeTile(Pos p);
	static bool isAdjacentToWall(Pos p);

	static bool crossesWall(Pos from, Pos to);

private:
	// Recalculate the entire hueristic map(called when the target changes)
	void recalculateHueristicMap();

	// Helper methods to calculate hueristic based on chosen type
	double calculateHueristic(int x, int y) const;
	double calculateHueristic(Pos pos) const;

	//Stores current target to calculate hueristic map
	Pos m_target;

	//Precomputed hueristic map for the grid (recomputed when the target changes)
	std::array<std::array<double, TILE_COUNT_X>, TILE_COUNT_Y> m_hueristicMap;

	//Cost map for the grid (extra cost for edge tiles/bumpy tiles)
	std::array<std::array<double, TILE_COUNT_X>, TILE_COUNT_Y> m_costMap;

	// Additional cost map that can be used to add extra cost to certain tiles (e.g., other robots)
	std::array<std::array<double, TILE_COUNT_X>, TILE_COUNT_Y> m_additionalCostMap;
};

