//
// Created by mjj on 17-7-12.
//

#include <QApplication>
#include <ros/ros.h>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    QApplication a(argc, argv);
    MainWindow w(node);
    w.show();

    return a.exec();
}
