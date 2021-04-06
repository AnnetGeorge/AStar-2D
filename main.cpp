#include "src/AStar.hpp"

int main()
{
    AStar AStar;
    AStar.setDefaultMap();
    GridLoc start = {1,1};
    GridLoc goal = {45,25};

    if (AStar.checkCollision(start)){
        std::cout << "The START location is in collision or Out of Bounds!" << std::endl;
    }
    else if (AStar.checkCollision(goal)){
        std::cout << "The GOAL location is in collision or Out of Bounds!" << std::endl;
    }
    else{
        std::cout << "Generating Path with AStar..." << std::endl;
        try{
            auto path = AStar.findPath(start, goal);
            AStar.write2file(path);
            std::cout << "Result written to file output_path.txt in current directory" << std::endl;
        }
        catch (bool foundPath){
            std::cout << "THERE IS NO FEASIBLE PATH BETWEEN START AND GOAL." << std::endl;
        }
    }
}