//
// Created by zzy on 3/5/18.
//
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <fstream>

static std::ofstream g_outfile;
static int next_signal = 0;

void saveIndex(const std_msgs::String file_name_msg)
{
    g_outfile << file_name_msg << std::endl;
}

void publishPCD()
{

}

int main(int argc, char *argv[])
{
    g_outfile.open( "../save_data/index/save_index.txt", std::ios::app);
    if( !g_outfile )
        std::cerr << "Fail to open txt file!" << std::endl;

    ros::init(argc, argv, "listen_index_node");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe( "rslidar_points", 1, saveIndex );

    ros::spin();
    g_outfile.close();
    return 0;
}
