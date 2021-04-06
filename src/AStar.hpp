#ifndef __ASTAR_2D_
#define __ASTAR_2D_

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <algorithm>


struct GridLoc
{
    int x, y;
};

/* Hash function for GridLoc */
namespace std {
    template <> struct hash<GridLoc> {
        std::size_t operator()(const GridLoc& id) const noexcept {
            return std::hash<int>()(id.x ^ id.y);
        }
    };
}

using uint = unsigned int;
using CoordinateList = std::vector<GridLoc>;

class AStar
{
public:
    AStar();
    void setDefaultMap();
    bool checkCollision(GridLoc coordinates_);
    void setGridDims(GridLoc gridDims_);
    void enableDiagonal(bool eightConnected_);
    CoordinateList findPath(GridLoc start_, GridLoc goal_);
    void addObstacle(GridLoc coordinates_);
    void removeObstacle(GridLoc coordinates_);
    static uint eucDistance(GridLoc source_, GridLoc target_);
    void write2file(CoordinateList &path_);

private:
    CoordinateList neighDirections, obs;
    GridLoc gridDims;
    uint connectedness; // 8 or 4 point connectedness
    bool foundPath;

    /* unordered maps used to update parent node and best cost to node */
    std::unordered_map <GridLoc, GridLoc> parentLoc;
    std::unordered_map <GridLoc, uint> gCost;

    /* priority queue used to keep track of next node to explore in the frontier */
    typedef std::pair<uint, GridLoc> FNode;
    std::priority_queue <FNode, std::vector<FNode>, std::greater<FNode>> frontier;
};

#endif