

#pragma once

#include <Eigen/Core>

namespace ins_slam{

struct Bbox
{
    Eigen::Vector3d center;
    Eigen::Vector3d dimension;
    double theta = 0.0;
    int label = -1;
    double score = 0.0;
};

struct Graph
{
    std::vector<int> node_labels;
    std::vector<Eigen::Vector3d> node_centers;
    std::vector<Eigen::Vector3d> node_dimensions;
    std::vector<std::vector<int>> node_desc; // 节点描述子
    std::vector<std::pair<int,int>> edges;
    std::vector<double> edge_value;
    std::vector<double> edge_weights;
};


}


