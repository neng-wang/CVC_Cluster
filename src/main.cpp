
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <SemanticCluster.hpp>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


bool remap=true;

std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> loadCloud(std::string file_cloud, std::string file_label){

    auto data_cfg = YAML::LoadFile("../config/config.yaml");
    std::vector<int> label_map;
    YAML::Node learning_map;
    learning_map = data_cfg["learning_map"];
    label_map.resize(260);
    for (auto it = learning_map.begin(); it != learning_map.end(); ++it)
    {
        label_map[it->first.as<int>()] = it->second.as<int>();
    }

    std::ifstream in_label(file_label, std::ios::binary);
    if (!in_label.is_open()) {
        std::cerr << "No file:" << file_label << std::endl;
        exit(-1);
    }
    in_label.seekg(0, std::ios::end);
    uint32_t num_points = in_label.tellg() / sizeof(uint32_t);
    in_label.seekg(0, std::ios::beg);
    std::vector<uint32_t> values_label(num_points);
    in_label.read((char*)&values_label[0], num_points * sizeof(uint32_t));
    std::ifstream in_cloud(file_cloud, std::ios::binary);
    std::vector<float> values_cloud(4 * num_points);
    in_cloud.read((char*)&values_cloud[0], 4 * num_points * sizeof(float));


    std::vector<Eigen::Vector3d> pc_out(num_points);
    std::vector<int> label_out(num_points);

    for (uint32_t i = 0; i < num_points; ++i) {
        uint32_t sem_label;
        if (remap) {
            sem_label = label_map[(int)(values_label[i] & 0x0000ffff)];
        } 
        else {
            sem_label = values_label[i];
        }
        if (sem_label == 0) {
            pc_out[i] = Eigen::Vector3d(0,0,0);
            label_out[i] = 0;
            continue;
        }
        pc_out[i] = Eigen::Vector3d(values_cloud[4 * i],values_cloud[4 * i + 1],values_cloud[4 * i + 2]);
        label_out[i] = (int) sem_label;
    }
    in_label.close();
    in_cloud.close();
    return std::make_pair(pc_out,label_out);
}
    
std::pair<std::vector<Eigen::Vector3d>,std::vector<int>>  transformpc(const std::pair<std::vector<Eigen::Vector3d>,std::vector<int>>  &frame, const Eigen::Isometry3d &trans_matrix){
        std::vector<Eigen::Vector3d> pc_out(frame.first.size());
        for(int i=0;i<frame.first.size();i++){
            pc_out[i] = trans_matrix.linear()*frame.first[i] + trans_matrix.translation();
        }
        return std::make_pair(pc_out,frame.second);
}

int main(int argc, char **argv){

    std::string conf_file = "../config/config.yaml";
    if (argc > 1)
    {
        conf_file = argv[1];
    }
    auto data_cfg = YAML::LoadFile(conf_file);
    auto file_name_length=data_cfg["file_name_length"].as<int>();
    auto cloud_path = data_cfg["cloud_file1"].as<std::string>();
    auto label_path = data_cfg["label_file1"].as<std::string>();

    bool cluster_view = data_cfg["cluster_view"].as<bool>();
    // cluster A R P: 2，1, 2回环检测比较好
    double deltaA = data_cfg["deltaA"].as<double>(); //z轴夹角分辨率,单位：度
    double deltaR = data_cfg["deltaR"].as<double>(); //xoy平面的半径，单位:m
    double deltaP = data_cfg["deltaP"].as<double>(); //与x轴正方向的夹角分辨率，单位:度
    
    auto cloud = loadCloud(cloud_path,label_path);

    // 不能先转换坐标，否则不对
    // Eigen::Isometry3d trans_matrix = Eigen::Isometry3d::Identity();
    // trans_matrix.translation() << 370.32, 4.06, 8.0;
    // trans_matrix.linear() << 0.9988, 0.0471, -0.00458, -0.047, 0.99865, 0.0218, 0.0056, 0.021579, 0.99975;

    // auto cloud = transformpc(cloud_in,trans_matrix);
    // down smaple
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_pc(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_fliter(new pcl::PointCloud<pcl::PointXYZL>);
    for(int i=0;i<cloud.first.size();i++){
        pcl::PointXYZL point_l;
        point_l.x = cloud.first[i][0]; point_l.y = cloud.first[i][1]; point_l.z = cloud.first[i][2];
        point_l.label = cloud.second[i];
        cloud_pc->emplace_back(point_l);
    }
    pcl::VoxelGrid<pcl::PointXYZL> voxel_filter;
    voxel_filter.setInputCloud(cloud_pc);              
    voxel_filter.setLeafSize(0.2, 0.2, 0.2);  
    voxel_filter.filter(*cloud_fliter);

    std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> cloud_input;
    for(int i=0;i<cloud_fliter->size();i++){
        cloud_input.first.emplace_back(Eigen::Vector3d((*cloud_fliter)[i].x,(*cloud_fliter)[i].y,(*cloud_fliter)[i].z));
        cloud_input.second.emplace_back((*cloud_fliter)[i].label);
    }
    std::cout<<"pc raw size:"<<cloud.first.size()<<std::endl;
    std::cout<<"label downsample size:"<<cloud_input.first.size()<<std::endl;
    auto start_time = std::chrono::steady_clock::now();


    auto cluster_box = ins_slam::Cluster(cloud.first,cloud.second,deltaA,deltaR,deltaP,cluster_view);
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std::cout<<"Total time consumption:"<<duration<<"ms"<<std::endl;
        
    std::cout<<"cluster_box size:"<<cluster_box.size()<<std::endl;

    return 0;
}