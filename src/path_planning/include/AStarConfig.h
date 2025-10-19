#pragma once
#include <vector>

//Unfortnately, these have to live here to be considered "constants"
constexpr int TILE_COUNT_X = (12000 / 500);//24
constexpr int TILE_COUNT_Y = (8000 / 500);//16

//Converts each meter to 25 pixels
constexpr int SCREEN_DIMENTION_X = (1000 / 25) * TILE_COUNT_X;
constexpr int SCREEN_DIMENTION_Y = (1000 / 25) * TILE_COUNT_Y;

extern const float COST_DEFAULT;
extern const float COST_EDGE_OF_FIELD;
extern const float COST_ADJACENT_TO_WALL;
extern const float COST_ROUGH_TERRAIN;

enum class HeuristicType {
	Euclidean,
	Manhattan,
	Chebyshev
};
extern HeuristicType CHOSEN_HURISTIC;

struct Connection {
	int fromX;
	int fromY;
	int toX;
	int toY;
};
struct Wall {
	int fromX;
	int fromY;
	int toX;
	int toY;
};

extern const std::vector<Connection> excludedConnections;
extern const std::vector<Wall> walls;
extern const std::vector<Wall> roughTerrain;