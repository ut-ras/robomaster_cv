#include "AStarConfig.h"

//Costs are additive (at least for now)
const float COST_DEFAULT = 1.0;
const float COST_EDGE_OF_FIELD = 0.65;
const float COST_ADJACENT_TO_WALL = 0.75;
const float COST_ROUGH_TERRAIN = 3.0;
const float COST_RAISED_TERRAIN = 2.0;

HeuristicType CHOSEN_HURISTIC = HeuristicType::Euclidean;

//All indexes are inclusive and 0-based
const std::vector<Connection> excludedConnections = {};
const std::vector<Wall> walls = {
	Wall({8, 11, 9, 24}),
	Wall({12, 8, 13, 16}),
	//Flip over center y-axis (don't change y values)
	Wall({(TILE_COUNT_X - 1) - 9, 11, (TILE_COUNT_X - 1) - 8, 24}),
	Wall({(TILE_COUNT_X - 1) - 13, 8, (TILE_COUNT_X - 1) - 12, 16}),

	Wall({9, 7, 16, 8}),
	//Flip over center y-axis (don't change y values)
	Wall({(TILE_COUNT_X - 1) - 16, 7, (TILE_COUNT_X - 1) - 9, 8}),
	
	Wall({15, 3, 24, 4}),
	Wall({24, 3, 33, 4}),

	Wall({19, 25, 20, 29}),
	Wall({(TILE_COUNT_X - 1) - 20, 25, (TILE_COUNT_X - 1) - 19, 29}),
	Wall({20, 24, (TILE_COUNT_X - 1) - 20, 25}),
	Wall({20, 28, (TILE_COUNT_X - 1) - 20, 29}),
};
const std::vector<TerrainWeight> roughTerrain = {
	TerrainWeight({ 1,23, 18, 32 , COST_ROUGH_TERRAIN}),
	TerrainWeight({ 1,23, 19, 28 , COST_ROUGH_TERRAIN}),
	TerrainWeight({ 1,23, 18, 25 , COST_ROUGH_TERRAIN}),

	//Flip over center y-axis (don't change y values)
	TerrainWeight({ (TILE_COUNT_X - 1) - 18, 23, (TILE_COUNT_X - 1) - 1, 32 , COST_ROUGH_TERRAIN}),
	TerrainWeight({ (TILE_COUNT_X - 1) - 19, 23, (TILE_COUNT_X - 1) - 1, 28 , COST_ROUGH_TERRAIN}),
	TerrainWeight({ (TILE_COUNT_X - 1) - 18, 23, (TILE_COUNT_X - 1) - 1, 25 , COST_ROUGH_TERRAIN}),

	TerrainWeight({9,0,16,7, COST_RAISED_TERRAIN}),
	TerrainWeight({17,0,24,4, COST_RAISED_TERRAIN}),
	//Flip over center y-axis (don't change y values)
	TerrainWeight({(TILE_COUNT_X - 1) - 24, 0, (TILE_COUNT_X - 1) - 17, 4, COST_RAISED_TERRAIN}),
	TerrainWeight({(TILE_COUNT_X - 1) - 16, 0, (TILE_COUNT_X - 1) - 9, 7, COST_RAISED_TERRAIN}),
};