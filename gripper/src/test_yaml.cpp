#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "simulation.h"

int main(int argc, char** argv) {
    
    if (argc != 2) {
        std::cout<<"Give me the filename!\n";
        return 1;
    }
    
    std::ifstream fin(argv[1]);
    YAML::Parser parser(fin);
    
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    const YAML::Node& node = doc[0];
    Simuation sim;
    node >> sim;
    
    std::cout<<"Simulation with "<<sim.trajectory.size()<<" elements\n";
    std::cout<<"Pre box dims: "<<sim.pre_box_dims.getX()<<", "
                               <<sim.pre_box_dims.getY()<<", "
                               <<sim.pre_box_dims.getZ()<<"\n";
    std::cout<<"Post box dims: "<<sim.post_box_dims.getX()<<", "
                               <<sim.post_box_dims.getY()<<", "
                               <<sim.post_box_dims.getZ()<<"\n";
    
    
    return 0;
}