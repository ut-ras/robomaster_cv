#include "Grid.h"
#include <iostream>
#include <iomanip>

Grid::Grid() : m_target(), m_hueristicMap(), m_costMap(), m_additionalCostMap()
{
	// Initialize maps
	for (int y = 0; y < TILE_COUNT_Y; ++y)
	{
		for (int x = 0; x < TILE_COUNT_X; ++x)
		{
			m_hueristicMap[y][x] = 0.0;
			m_additionalCostMap[y][x] = 0.0;
			m_costMap[y][x] = COST_DEFAULT; // Default cost is 1.0

			if (isEdgeTile({x, y}))
			{
				m_costMap[y][x] += COST_EDGE_OF_FIELD; // Cost for edge tiles
			}
			if (isRoughTerrain({x, y}))
			{
				m_costMap[y][x] += getRoughTerrain({x, y}); // Cost for rough terrain
			}
			if (isAdjacentToWall({x, y}))
			{
				m_costMap[y][x] += COST_ADJACENT_TO_WALL; // Cost for tiles adjacent to walls
			}
		}
	}

	// blurCostMap();
}

void Grid::blurCostMap()
{
	// Apply blur to each tile's cost based on surrounding tiles to create smoother transitions
	//  Simple box blur
	std::array<std::array<double, TILE_COUNT_X>, TILE_COUNT_Y> blurredCostMap = m_costMap;
	for (int y = 0; y < TILE_COUNT_Y; ++y)
	{
		for (int x = 0; x < TILE_COUNT_X; ++x)
		{
			double totalCost = 0.0;
			int count = 0;
			// Sum costs of neighboring tiles including itself
			for (int dy = -1; dy <= 1; ++dy)
			{
				for (int dx = -1; dx <= 1; ++dx)
				{
					if (dx == dy) // Skip diagonals for simplicity
						continue;

					int nx = x + dx;
					int ny = y + dy;
					if (nx >= 0 && nx < TILE_COUNT_X && ny >= 0 && ny < TILE_COUNT_Y)
					{
						totalCost += m_costMap[ny][nx];
						++count;
					}
				}
			}
			// Average the cost
			blurredCostMap[y][x] = totalCost / count;
		}
	}
	m_costMap = blurredCostMap;
}

bool Grid::isEdgeTile(Pos p)
{
	return (p.x == 0 || p.x == TILE_COUNT_X - 1 ||
			p.y == 0 || p.y == TILE_COUNT_Y - 1);
}

bool Grid::isAdjacentToWall(Pos p)
{
	for (size_t i = 0; i < walls.size(); i++)
	{
		if (walls[i].fromX <= p.x && walls[i].toX >= p.x && walls[i].fromY <= p.y && walls[i].toY >= p.y)
		{
			return true;
		}
	}
	return false;
}

bool Grid::isRoughTerrain(Pos p)
{
	for (size_t i = 0; i < roughTerrain.size(); i++)
	{
		if (roughTerrain[i].fromX <= p.x && roughTerrain[i].toX >= p.x && roughTerrain[i].fromY <= p.y && roughTerrain[i].toY >= p.y)
		{
			return true;
		}
	}
	return false;
}

float Grid::getRoughTerrain(Pos p)
{
	for (size_t i = 0; i < roughTerrain.size(); i++)
	{
		if (roughTerrain[i].fromX <= p.x && roughTerrain[i].toX >= p.x && roughTerrain[i].fromY <= p.y && roughTerrain[i].toY >= p.y)
		{
			return roughTerrain[i].weight;
		}
	}
	return 0.0f;
}

bool Grid::crossesWall(Pos from, Pos to)
{
	for (Wall w : walls)
	{
		if ((from.y == w.fromY && to.y == w.toY) || (to.y == w.fromY && from.y == w.toY))
		{
			if ((from.x >= w.fromX && to.x <= w.toX) || (to.x >= w.fromX && from.x <= w.toX))
			{
				return true;
			}
		}
		if ((from.x == w.fromX && to.x == w.toX) || (to.x == w.fromX && from.x == w.toX))
		{
			if ((from.y >= w.fromY && to.y <= w.toY) || (to.y >= w.fromY && from.y <= w.toY))
			{
				return true;
			}
		}
	}
	return false;
}

void Grid::recalculateHueristicMap()
{

	for (int y = 0; y < TILE_COUNT_Y; ++y)
	{
		for (int x = 0; x < TILE_COUNT_X; ++x)
		{
			m_hueristicMap[y][x] = calculateHueristic(x, y);
		}
	}
}

void Grid::setTarget(int x, int y)
{
	if (x != m_target.x || y != m_target.y)
	{
		m_target = {x, y};
		recalculateHueristicMap();
	}
}

void Grid::setTarget(Pos target)
{
	setTarget(target.x, target.y);
}

Pos Grid::getTarget()
{
	return m_target;
}

double Grid::getHueristic(int x, int y) const
{
	if (x >= 0 && x < TILE_COUNT_X && y >= 0 && y < TILE_COUNT_Y)
	{
		return m_hueristicMap[y][x];
	}
	return 0.0;
}

void Grid::setHueristic(int x, int y, double hueristic)
{
	if (x >= 0 && x < TILE_COUNT_X && y >= 0 && y < TILE_COUNT_Y)
	{
		m_hueristicMap[y][x] = hueristic;
	}
}

double Grid::getCost(int x, int y) const
{
	if (x >= 0 && x < TILE_COUNT_X && y >= 0 && y < TILE_COUNT_Y)
	{
		return m_costMap[y][x];
	}
	return 0.0;
}

void Grid::setCost(int x, int y, double cost)
{
	if (x >= 0 && x < TILE_COUNT_X && y >= 0 && y < TILE_COUNT_Y)
	{
		m_costMap[y][x] = cost;
	}
}

double Grid::getAdditionalCost(int x, int y) const
{
	if (x >= 0 && x < TILE_COUNT_X && y >= 0 && y < TILE_COUNT_Y)
	{
		return m_additionalCostMap[y][x];
	}
	return 0.0;
}

void Grid::setAdditionalCost(int x, int y, double cost)
{
	if (x >= 0 && x < TILE_COUNT_X && y >= 0 && y < TILE_COUNT_Y)
	{
		m_additionalCostMap[y][x] = cost;
	}
}

void Grid::addCostBox(int x, int y, double centerCost, double adjacentCost)
{
	setAdditionalCost(x, y, centerCost);
	for (int dx = -1; dx <= 1; ++dx)
	{
		for (int dy = -1; dy <= 1; ++dy)
		{
			if (dx == 0 && dy == 0)
				continue; // Skip center tile
			setAdditionalCost(x + dx, y + dy, adjacentCost);
		}
	}
}

double Grid::calculateHueristic(int x, int y) const
{
	switch (CHOSEN_HURISTIC)
	{
	case HeuristicType::Euclidean:
		return std::sqrt(std::pow(m_target.x - x, 2) + std::pow(m_target.y - y, 2));
		break;
	case HeuristicType::Manhattan:
		return std::abs(m_target.x - x) + std::abs(m_target.y - y);
		break;
	case HeuristicType::Chebyshev:
		return std::max(std::abs(m_target.x - x), std::abs(m_target.y - y));
		break;
	default:
		return 0.0;
		break;
	}
}
double Grid::calculateHueristic(Pos pos) const
{
	return calculateHueristic(pos.x, pos.y);
}
