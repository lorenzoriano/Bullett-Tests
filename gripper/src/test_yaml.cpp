#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "simulation.h"

#include "simulation.h"
#include <btBulletDynamicsCommon.h>

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
    Simulation sim;
    node >> sim;
    
    std::cout<<"Simulation with "<<sim.trajectory.size()<<" elements\n";
    std::cout<<"Pre box dims: "<<sim.pre_box_dims.getX()<<", "
                               <<sim.pre_box_dims.getY()<<", "
                               <<sim.pre_box_dims.getZ()<<"\n";
    std::cout<<"Post box dims: "<<sim.post_box_dims.getX()<<", "
                               <<sim.post_box_dims.getY()<<", "
                               <<sim.post_box_dims.getZ()<<"\n";
    
    sim.setup_times();
    std::cout<<"Simulation ends at: "<< sim.trajectory[sim.trajectory.size()-1].time<<"\n";
    std::cout<<"Testing the timing\n";
    
    btClock clock;
    int step = 0;
    double first_tick;
    first_tick = 0;
    while (step < sim.trajectory.size()) {
        double next_tick = sim[step].time;        
        double delta_tick = next_tick - first_tick;
        double elapsed = clock.getTimeMicroseconds() * 1.e-6;
        
        if (elapsed < delta_tick) {
            usleep(5);
            continue;
        }
        std::cerr<<"Step: "<<step<<", Elapsed: "<<elapsed<<", delta_tick: "<<delta_tick<<"\n";
        
        clock.reset();
        step++;    
        first_tick = next_tick;
        
    }
    return 0;
}