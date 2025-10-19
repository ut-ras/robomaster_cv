#include "AStarConfig.h"

//Costs are additive (at least for now)
const float COST_DEFAULT = 1.0;
const float COST_EDGE_OF_FIELD = 0.25;
const float COST_ADJACENT_TO_WALL = 0.5;
const float COST_ROUGH_TERRAIN = 3.0;

HeuristicType CHOSEN_HURISTIC = HeuristicType::Euclidean;

//All indexes are inclusive and 0-based
const std::vector<Connection> excludedConnections = {};
const std::vector<Wall> walls = {
	Wall({3, 0, 4, 7}),
	Wall({4, 7, 5, 8}),
	Wall({5, 4, 6, 9}),
	Wall({6, 3, 9, 4}),
	Wall({11, 0, 12, 2}),
	////Rotate x&y 180 for opposite side (-x, -y), swap from&to
	Wall({ (TILE_COUNT_X - 1) - 4, (TILE_COUNT_Y - 1) - 7, (TILE_COUNT_X - 1) - 3, (TILE_COUNT_Y - 1) - 0 }),
	Wall({ (TILE_COUNT_X - 1) - 5, (TILE_COUNT_Y - 1) - 8, (TILE_COUNT_X - 1) - 4, (TILE_COUNT_Y - 1) - 7 }),
	Wall({ (TILE_COUNT_X - 1) - 6, (TILE_COUNT_Y - 1) - 9, (TILE_COUNT_X - 1) - 5, (TILE_COUNT_Y - 1) - 4 }),
	Wall({ (TILE_COUNT_X - 1) - 9, (TILE_COUNT_Y - 1) - 4, (TILE_COUNT_X - 1) - 6, (TILE_COUNT_Y - 1) - 3 }),
	Wall({ (TILE_COUNT_X - 1) - 12, (TILE_COUNT_Y - 1) - 2, (TILE_COUNT_X - 1) - 11, (TILE_COUNT_Y - 1) - 0 }),
};
const std::vector<Wall> roughTerrain = {
	Wall({ 3, 10, 5, 13 }),
	Wall({ 6, 4, 9, 13 }),
	//Rotate x&y 180 for opposite side (-x, -y), swap from&to
	Wall({ (TILE_COUNT_X - 1) - 5, (TILE_COUNT_Y - 1) - 13, (TILE_COUNT_X - 1) - 3, (TILE_COUNT_Y - 1) - 10, }),
	Wall({ (TILE_COUNT_X - 1) - 9, (TILE_COUNT_Y - 1) - 13, (TILE_COUNT_X - 1) - 6, (TILE_COUNT_Y - 1) - 4, }),
};