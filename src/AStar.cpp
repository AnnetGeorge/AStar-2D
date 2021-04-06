#include "AStar.hpp"

/* Helper functions for GridLoc */
bool operator == (const GridLoc& a, const GridLoc& b) {
  return a.x == b.x && a.y == b.y;
}

bool operator != (GridLoc a, GridLoc b) {
  return !(a == b);
}

GridLoc operator + (const GridLoc& a_, const GridLoc& b_)
{
    return{ a_.x + b_.x, a_.y + b_.y };
}

bool operator < (GridLoc a, GridLoc b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, const GridLoc& loc) {
  out << '(' << loc.x << ',' << loc.y << ')';
  return out;
}

/* Constructor for AStar class */
AStar::AStar()
{
    neighDirections = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
    foundPath = false;
}

/* Default map of size [50,30] with obstacle walls */
void AStar::setDefaultMap()
{
    int xDim = 50;
    int yDim = 30;
    setGridDims({xDim,yDim});
    enableDiagonal(true);

    // Set boundaries
    for (int i=0; i <= xDim; i++){
        addObstacle({i, 0});
        addObstacle({i, yDim});
    }

    for (int i=0; i <= yDim; i++){
        addObstacle({0, i});
        addObstacle({xDim, i});
    }

    // Set obstacle walls
    for (int i=10; i <= 20; i++){
        addObstacle({i, 15});
    }
    for (int i=0; i < 15; i++){
        addObstacle({20, i});
    }

   for (int i=15; i < 30; i++){
        addObstacle({30, i});
    }
   for (int i=0; i < 16; i++){
        addObstacle({40, i});
    }
}

/* Set grid world dimensions */
void AStar::setGridDims(GridLoc gridDims_)
{
    gridDims = gridDims_;
}

/* Enable diagonal movement using 8 point adjacency */
void AStar::enableDiagonal(bool eightConnected_)
{
    connectedness = (eightConnected_ ? 8 : 4);
}

/* Add obstacles to the grid world at specified coordinates */
void AStar::addObstacle(GridLoc coordinates_)
{
    obs.push_back(coordinates_);
}

/* Remove obstacles from the grid world at specified coordinates */
void AStar::removeObstacle(GridLoc coordinates_)
{
    auto it = std::find(obs.begin(), obs.end(), coordinates_);
    if (it != obs.end()) {
        obs.erase(it);
    }
}

/* Runs AStar search using specified start and goal coordinates and returns path */
CoordinateList AStar::findPath(GridLoc start_, GridLoc goal_)
{
    // initialisation
    frontier.emplace(std::make_pair(0, start_));
    parentLoc[start_] = start_;
    gCost[start_] = 0;

    while (!frontier.empty()){
        GridLoc curr = frontier.top().second;

        if (curr == goal_){
            foundPath = true;
            break;
        }
        frontier.pop();

        // check all neighbours and add to frontier with priority fcost (if necessary)
        for (uint i = 0; i < connectedness; i++) {
            GridLoc newCoordinates(curr + neighDirections[i]);
            if (checkCollision(newCoordinates)) {
                continue;
            }
            uint totalCost = gCost[curr] + eucDistance(curr, newCoordinates);

            if (gCost.find(newCoordinates) == gCost.end() || totalCost < gCost[newCoordinates]){
                gCost[newCoordinates] = totalCost;
                uint fCost= totalCost + eucDistance(newCoordinates, goal_);
                frontier.emplace(fCost, newCoordinates);
                parentLoc[newCoordinates] = curr;
            }
        }
    }

    // Throw error if no path is found between START and GOAL
    if (!foundPath){
        throw foundPath;
    }

    // Reconstruct path from START to GOAL and return
    CoordinateList path;
    GridLoc current = goal_;
    while (current != start_) {
        path.push_back(current);
        GridLoc new_current = parentLoc[current];
        current = new_current;
    }
    path.push_back(start_);
    std::reverse(path.begin(), path.end());
    return path;
}

/* Collision check for given coordinates */
bool AStar::checkCollision(GridLoc coordinates_)
{
    if (std::find(obs.begin(), obs.end(), coordinates_) != obs.end() ||
        coordinates_.x < 0 || coordinates_.x >= gridDims.x ||
        coordinates_.y < 0 || coordinates_.y >= gridDims.y){
            return true;
        }
    return false;
}

/* Euclidean distance for g and h cost calculations*/
uint AStar::eucDistance(GridLoc source_, GridLoc target_)
{
    // uses euclidean heuristic
    auto deltaX = abs(source_.x - target_.x);
    auto deltaY = abs(source_.y - target_.y);
    return static_cast<uint>(10 * sqrt(pow(deltaX, 2) + pow(deltaY, 2)));
}

/* Write output path coordinates to a file*/
void AStar::write2file(CoordinateList &path_)
{
    std::ofstream file;
    file.open("output_path.txt");
    for (uint i=0; i < path_.size(); i++){
        file << path_[i] << std::endl;
    }
    file.close();
}
