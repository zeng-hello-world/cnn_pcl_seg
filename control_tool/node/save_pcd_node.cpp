//
// Created by mjj on 17-7-12.
//

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <queue>

std::queue<pcl::PointCloud<pcl::PointXYZI> > g_frames;
int g_max_frames = 10;
ros::Publisher pub_frame;
int g_index = 0;
std::string g_save_path;

template<typename T>
std::string num2str(T num)
{
    std::stringstream ss;
    std::string st;
    ss <<std::fixed<< num;
    ss >> st;
    return st;
}

void fullscanCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr msg){
    if (msg->header.seq == 0)return;
    if (msg->empty())return;
    while(g_frames.size() > g_max_frames){
        g_frames.pop();
    }
    g_frames.push(*msg);
}

void frameCallback(const std_msgs::String msg){
    if (g_frames.empty())
        return;
    std::string tmp_str = msg.data;
    if (tmp_str == "pre_frame"){
        std::cout<<"press pre_frame button"<<std::endl;
        g_index++;
        if (g_index >= g_frames.size() - 1){
            g_index = g_frames.size() - 1;
        }
    }else if(tmp_str == "next_frame"){
        std::cout<<"press next_frame button"<<std::endl;
        g_index--;
        if (g_index <= 0){
            g_index = 0;
        }
    }
    int pop_times = g_frames.size() - g_index - 1;
    std::queue<pcl::PointCloud<pcl::PointXYZI> > tmp_queue = g_frames;
    for (int i = 0; i < pop_times; ++i) {
        tmp_queue.pop();
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    *tmp_cloud_ptr = tmp_queue.front();
    tmp_cloud_ptr->header.seq = 0;
    pub_frame.publish(tmp_cloud_ptr);

    if (tmp_str == "save_frame"){
        std::cout<<"press save_frame button"<<std::endl;
        double timestamp = tmp_cloud_ptr->header.stamp;
        std::string tmp_file_path = g_save_path + num2str(timestamp) + ".pcd";
        pcl::io::savePCDFileBinary(tmp_file_path.c_str(), *tmp_cloud_ptr);
    }
}

void frameNumCallback(const std_msgs::String msg){
    std::string tmp_str = msg.data;
    g_max_frames = atoi(tmp_str.c_str());
}

void continueCallback(const std_msgs::String msg){
    g_index = 0;
}


///-------------------------------------------------------------------------------------------
///----------------------------------- for PCD files -----------------------------------------
///-------------------------------------------------------------------------------------------
static pcl::PCLHeader pt_header;
enum recv_sign { none_sign = 0, stop_sign = 1, continue_sign = 2, load_sign = 3, change_sign = 4 };
static int g_con_signal = none_sign;
static std::string g_pcd_info;
static int g_cur_index = 0;
static pcl::PointCloud<pcl::PointXYZI>::Ptr cur_pcd_ptr (new pcl::PointCloud<pcl::PointXYZI>);
static ros::Publisher g_pub_cloud;
static ros::Publisher g_pub_cur_index;
static std::string g_file_path;
static std::vector<std::string> g_pcd_filelist;
static std::ofstream g_outfile;

void setSignal(const std_msgs::String& input_sign)
{
    std::string tmp = input_sign.data;
    if ( tmp == "0" )
        g_con_signal = 0;
    else if ( tmp == "1" )
        g_con_signal = 1;
    else if ( tmp == "2" )
        g_con_signal = 2;
    else if ( tmp == "3" )
        g_con_signal = 3;
}

void callbackOrderMsg(const std_msgs::String& input_msg)
{
    g_pcd_info = input_msg.data;

    std::cout << "receive sig: " << g_pcd_info << std::endl;
}

void splitPathStr(std::string& path_str)
{
    g_pcd_filelist.clear();
    std::stringstream sstr( path_str );
    std::string token;
    while(getline(sstr, token, ';'))
    {
        g_pcd_filelist.push_back(token);
    }
}

void publishPCD()
{
    pt_header.frame_id = "rslidar";

    std::string tmp_pcd_file_path = g_pcd_filelist[g_cur_index];

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (tmp_pcd_file_path, *cur_pcd_ptr) == -1)
        return;

    cur_pcd_ptr->header = pt_header;
    g_pub_cloud.publish(cur_pcd_ptr);

    std_msgs::String cur_index_msg;
    cur_index_msg.data = g_cur_index;
    g_pub_cur_index.publish( cur_index_msg );

    std::cout << "INFO: " << g_pcd_filelist[g_cur_index] << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listen_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub_fullscan = node.subscribe("/rslidar_points",10,fullscanCallback);
    ros::Subscriber sub_framenode = node.subscribe("/frame",10,frameCallback);
    ros::Subscriber sub_framenumnode = node.subscribe("/frame_num",10,frameNumCallback);
    ros::Subscriber sub_clear = node.subscribe("/continual_signal",10,continueCallback);
    // pcd file
    g_pub_cloud = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("rslidar_points",1);
    g_pub_cur_index = node.advertise<std_msgs::String>("cur_index",1);// no subscriber
    ros::Subscriber sub_pcd_sign = node.subscribe("/pcd_con_signal",10, setSignal);
    ros::Subscriber sub_pcd_info = node.subscribe("/pcd_info",1, callbackOrderMsg);

    pub_frame = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("/rslidar_points",1);
//    ros::spin();
//    std::string tmp_save_path = ros::package::getPath("control_tool") + "/save_data/index/";
    std::string tmp_save_path = "/home/zzy/CLionProjects/ROS_Project/ws/src/auto_play/src/play/control_tool/save_data/index";
    g_save_path = tmp_save_path + "/save_index.txt";
    g_outfile.open( g_save_path.c_str(), std::ios::app);
    if( !g_outfile )
        std::cerr << "Fail to open txt file!" << std::endl;

    while(ros::ok())
    {
        if( g_con_signal == continue_sign )
        {
            if ( g_cur_index < g_pcd_filelist.size() - 1 )
            {
                publishPCD();
                g_cur_index++;
            }
            else
                g_cur_index = 0;
        }
        else if ( g_con_signal == stop_sign )
        {
            if ( g_pcd_info == "pre_pcd_signal" )
            {
                if ( g_cur_index > 0 )
                {
                    g_cur_index--;
                    publishPCD();
                }
                else
                    std::cerr << "Reach 1st file!!" << std::endl;
            }
            else if ( g_pcd_info == "next_pcd_signal" )
            {
                if ( g_cur_index < g_pcd_filelist.size() - 1 )
                    g_cur_index ++;
                else
                    g_cur_index = 0;

                publishPCD();
            }
            else if ( g_pcd_info == "save_index" )
            {
                g_outfile << g_pcd_filelist[g_cur_index] << std::endl;
            }
            g_con_signal = none_sign;
        }
        else if ( g_con_signal == load_sign )
        {
            g_cur_index = 0;
            g_file_path = g_pcd_info;
            splitPathStr( g_file_path );
            publishPCD();

            g_con_signal = none_sign;
        }
        else if ( g_con_signal == change_sign )
        {
            int tmp_index = atoi(g_pcd_info.c_str());
            if ( tmp_index >= 0 && tmp_index < g_pcd_filelist.size() - 1 )
            {
                g_cur_index - tmp_index;
                publishPCD();
            }
            else
                std::cout << "Index out of range!" << std::endl;

            g_con_signal = none_sign;
        }

        ros::spinOnce();
    }

    g_outfile.close();
    return 0;
}