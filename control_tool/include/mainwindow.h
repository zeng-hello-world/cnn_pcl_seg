//
// Created by mjj on 17-7-12.
//

#ifndef PROJECT_MAINWINDOW_H
#define PROJECT_MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <sstream>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle node, QWidget *parent = 0);
    ~MainWindow();
public slots:
    //control_node
    void on_pushButton_next_clicked();
    void on_pushButton_continue_clicked();
    void on_pushButton_pre_frame_clicked();
    void on_pushButton_next_frame_clicked();
    void on_pushButton_save_frame_clicked();
    void spinBox_nFrame_valueChanged(int);
    //replay_node
    void on_pushButton_load_pcd_clicked();
    void on_pushButton_pre_pcd_clicked();
    void on_pushButton_next_pcd_clicked();
    void on_pushButton_continue_pcd_clicked();
    void on_pushButton_save_index_clicked();
    void comboBox_pcd_changed(const QString &cur_pcd);

private:
    void setWidgetDisable(bool &is_file_empty);
    void publishString(const ros::Publisher& pub,const std::string& str);
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists,std::string type);
    //kitti
    void sort_filelists_kitti(std::vector<std::string>& filists);
    void publishPCD();
    void subCurIndex(std_msgs::String cur_index);
    int findIndex(const std::vector<std::string>& file_lists,const std::string& cur_file_name);

    template<typename T>
    std::string num2str(T num) {
        std::stringstream ss;
        std::string st;
        ss << num;
        ss >> st;
        return st;
    }

    int max_save_num_;
    ros::Publisher pub_control_;
    ros::Publisher pub_continue_;
    ros::Publisher pub_frame_;
    ros::Publisher pub_max_frame_num_;
    ros::Publisher pub_pcd_;
    // pcd file
    std::string file_path_str_;
    ros::Publisher pub_pcd_info_;
    ros::Publisher pub_pcd_con_signal_; // load or continue signal
    ros::Subscriber sub_cur_index_;

    QString QProPath_;
    std::vector<std::string> pcd_filelists_;
    int cur_pcd_index_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_pcd_ptr_;
    pcl::PCLHeader header_;

    bool is_file_empty_;

private:
    Ui::MainWindow *ui;
};

#endif //PROJECT_MAINWINDOW_H
