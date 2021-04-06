# AStar-2D
Implementation of AStar path planning in C++ for a 2D grid world.

### Requirements
Compiler must support C++11
CMake : On Ubuntu, install Cmake using:
    sudo snap install cmake -classic
  

The program make use of a default grid world shown below (the visualisation is simply for demonstration of the default grid world).
The map can be customised by changing the map size, adding new obstacles and deleting existing obstacles. 

To run the program using the default grid, use CMake:

    cd AStar-2D
    mkdir build
    cd build
    cmake ..
    make
    ./main
    
This outputs the coordinates of the result (path) in the file `output_path.txt`
