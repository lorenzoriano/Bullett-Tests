#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <vector>
#include <iostream>
#include <fstream>
#include "btBulletDynamicsCommon.h"
#include <yaml-cpp/yaml.h>

struct Pose {
    btVector3 position;
    btQuaternion orientation;
    
    btTransform getTransform() const {
        return btTransform(orientation, position);
    }
};

struct PoseStamped {
    double time;
    Pose pose;
};

struct Simulation {
    
    Simulation() {
        m_already_setup = false;
    }
    
    btVector3 pre_box_dims;
    btVector3 post_box_dims;
    
    Pose pre_box_pose;
    Pose post_box_pose;
    
    std::vector<PoseStamped> trajectory;
    bool m_already_setup;
    
    void setup_times() {
        if (m_already_setup) {
            std::cerr<<"Already setup!!\n";
            return;
        }
        std::cout<<"Adjusting the times\n";
        m_already_setup = true;
        
        if (trajectory.size() == 0)
            return;
        double init_time = trajectory[0].time;
        for (int i=0; i<trajectory.size(); i++) {
            trajectory[i].time = (trajectory[i].time - init_time) * 1e-9;
        }
    }
    
    
    PoseStamped& operator[](int i) {
        return trajectory[i];
    }
    const PoseStamped& operator[](int i) const {
        return trajectory[i];
    }
    
    static Simulation createFromYaml(const char* filename);
};


inline void operator>>(const YAML::Node& node, Pose& pose) {
    const YAML::Node& position_node = node["position"];
    
    float x, y, z, w;
    position_node[0] >> x;
    position_node[1] >> y;
    position_node[2] >> z;
    
    pose.position.setX(x);
    pose.position.setY(y);
    pose.position.setZ(z);
    
    const YAML::Node& orientation_node = node["orientation"];
    orientation_node[0] >> x;
    orientation_node[1] >> y;
    orientation_node[2] >> z;
    orientation_node[3] >> w;
    pose.orientation.setX(x);
    pose.orientation.setY(y);
    pose.orientation.setZ(z);    
    pose.orientation.setW(w);    
}

inline void operator>>(const YAML::Node& node, PoseStamped& pose) {
    node["time"] >> pose.time;
    node["pose"] >> pose.pose;
}

inline void operator>>(const YAML::Node& node, Simulation& sim) {
    {
        const YAML::Node& vec = node["pre_box_dims"];
        float x, y, z;
        
        vec[0] >> x;
        vec[1] >> y;
        vec[2] >> z;
        
        sim.pre_box_dims.setX(x);
        sim.pre_box_dims.setY(y);
        sim.pre_box_dims.setZ(z);
    }
    {
        const YAML::Node& vec = node["post_box_dims"];
        float x,y,z;
        
        vec[0] >> x;
        vec[1] >> y;
        vec[2] >> z;
        
        sim.post_box_dims.setX(x);
        sim.post_box_dims.setY(y);
        sim.post_box_dims.setZ(z);
    }
    node["pre_box_pose"] >> sim.pre_box_pose;
    node["post_box_pose"] >> sim.post_box_pose;
    node["trajectory"] >> sim.trajectory;        
      
}

inline Simulation Simulation::createFromYaml(const char* filename) {
    std::ifstream fin(filename);
    YAML::Parser parser(fin);
    
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    const YAML::Node& node = doc[0];
    Simulation sim;
    node >> sim;
    return sim;
}

#endif