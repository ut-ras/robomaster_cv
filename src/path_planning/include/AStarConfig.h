#pragma once
#include <vector>

// Unfortnately, these have to live here to be considered "constants"
constexpr int TILE_SIZE = 250;
constexpr int TILE_COUNT_X = (12000 / 250); // 48
constexpr int TILE_COUNT_Y = (8000 / 250);	// 32

// Converts each meter to 25 pixels
constexpr int SCREEN_DIMENTION_X = (1000 / 25) * TILE_COUNT_X;
constexpr int SCREEN_DIMENTION_Y = (1000 / 25) * TILE_COUNT_Y;

// Dunno What unit this is or how this calculation works
// Presumasbly, this is in tile-size units
constexpr float ROBOT_RADIUS = 0.5;

extern const float COST_DEFAULT;
extern const float COST_EDGE_OF_FIELD;
extern const float COST_ADJACENT_TO_WALL;
extern const float COST_ROUGH_TERRAIN;
extern const float COST_RAISED_TERRAIN;

extern const float COST_OTHER_ROBOT;
extern const float COST_OTHER_ROBOT_ADJACENT;

enum class HeuristicType
{
	Euclidean,
	Manhattan,
	Chebyshev
};
extern HeuristicType CHOSEN_HURISTIC;

struct Connection
{
	int fromX;
	int fromY;
	int toX;
	int toY;
};
struct Wall
{
	int fromX;
	int fromY;
	int toX;
	int toY;
};
struct TerrainWeight
{
	int fromX;
	int fromY;
	int toX;
	int toY;
	float weight;
};

extern const std::vector<Connection> excludedConnections;
extern const std::vector<Wall> walls;
extern const std::vector<TerrainWeight> roughTerrain;