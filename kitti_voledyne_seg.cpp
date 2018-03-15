//
// Created by zzy on 3/14/18.
//

#include <ctime>
#include "ros/ros.h"
#include "fcn_data_gen/ground_remove.h"

static ros::Publisher g_cloud_pub;
static std::vector<std::string> file_lists;

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

void readKittiPclBinData(std::string &in_file, std::string& out_file)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}


int main(int argc, char **argv)
{
//    ros::init(argc, argv, "ground_remove_test");
//    ros::NodeHandle n;
//    g_cloud_pub = n.advertise< pcl::PointCloud< pcl::PointXYZI > > ("point_chatter", 1);

    std::string bin_path = "/media/zzy/文档/RoboSense/Data/velodyne/binary/";
    std::string pcd_path = "/media/zzy/文档/RoboSense/Data/velodyne/pcd/";
    read_filelists( bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );
    for (int i = 0; i < file_lists.size(); ++i)
    {
        std::string bin_file = bin_path + file_lists[i];
        std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".pcd";
        std::string pcd_file = pcd_path + tmp_str;
        readKittiPclBinData( bin_file, pcd_file );
    }

    return 0;
}