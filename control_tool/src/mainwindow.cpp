//
// Created by mjj on 17-7-12.
// co zzy 18-3-5
//
#include <std_msgs/String.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle node, QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    max_save_num_ = ui->spinBox_nFrame->value();
    pub_control_ = node.advertise<std_msgs::String>("control_signal",1);
    pub_continue_ = node.advertise<std_msgs::String>("continual_signal",1);
    pub_frame_ = node.advertise<std_msgs::String>("frame",1);
    pub_max_frame_num_ = node.advertise<std_msgs::String>("frame_num",1);
    cur_pcd_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    header_.frame_id = "rslidar";
    pub_pcd_ = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("rslidar_points",1);
    // pcd file
    pub_pcd_info_ = node.advertise<std_msgs::String>("pcd_info",1);
    pub_pcd_con_signal_ = node.advertise<std_msgs::String>("pcd_con_signal",1);
//    sub_cur_index_ = node.subscribe<std_msgs::String>("/cur_index",10,subCurIndex);
    is_file_empty_ = true;
    this->setWidgetDisable( is_file_empty_ );

    connect(ui->spinBox_nFrame,SIGNAL(valueChanged(int)),this,SLOT(spinBox_nFrame_valueChanged(int)));
    connect(ui->comboBox_pcd, SIGNAL(activated(const QString&)), this, SLOT(comboBox_pcd_changed(const QString &)));
}

void MainWindow::comboBox_pcd_changed(const QString &cur_pcd){
    std::string tmp_pcd_file_name = cur_pcd.toStdString();
    int tmp_index = findIndex(pcd_filelists_,tmp_pcd_file_name);
    QString tmp_qstring;
    if (tmp_index == -1){
        ui->textEdit_msg->setText(tmp_qstring.fromStdString("error!! no" + tmp_pcd_file_name + " !"));
        return;
    }
    cur_pcd_index_ = tmp_index;

    // int to string
    std::string cur_index_str;
    std::stringstream stream;
    stream << cur_pcd_index_;
    cur_index_str = stream.str();
    if (pcd_filelists_.empty())
        return;

    publishString( pub_pcd_con_signal_, "4" );//change sign = 4
    publishString( pub_pcd_info_, cur_index_str );
}

int MainWindow::findIndex(const std::vector<std::string>& file_lists,const std::string& cur_file_name){
    int tmp_index = -1;
    if (file_lists.empty())return tmp_index;
    for (int i = 0; i < file_lists.size(); ++i) {
        if (file_lists[i] == cur_file_name){
            tmp_index = i;
            return tmp_index;
        }
    }
    return tmp_index;
}



void MainWindow::publishPCD()
{
    QString tmp_qstring;
    ui->comboBox_pcd->setCurrentText(tmp_qstring.fromStdString(pcd_filelists_[cur_pcd_index_]));
    std::string tmp_pcd_file_path = QProPath_.toStdString() + "/" + pcd_filelists_[cur_pcd_index_];
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (tmp_pcd_file_path.c_str(), *cur_pcd_ptr_) == -1) {
        ui->textEdit_msg->setText(tmp_qstring.fromStdString("failed load " + pcd_filelists_[cur_pcd_index_] + " !!"));
        return;
    }
    cur_pcd_ptr_->header = header_;
    pub_pcd_.publish(cur_pcd_ptr_);
}

void MainWindow::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type){
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
    // if voledyne, change sort mode
    if (dir_path.find("velodyne")!=std::string::npos)
    {
        this->sort_filelists_kitti(out_filelsits);
        ui->textEdit_msg->append( "Find velodyne path." );
    }
    else
    {
        this->sort_filelists(out_filelsits, type);
        ui->textEdit_msg->append( "Find rslidar path." );
    }
}

bool computePairNum(std::pair<double,std::string> pair1,std::pair<double,std::string> pair2)
{
    return pair1.first < pair2.first;
}

void MainWindow::sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;
    std::vector<std::pair<double,std::string> > filelists_pair;
    for (int i = 0; i < filists.size(); ++i) {
        std::string tmp_string = filists[i];
        int npos = tmp_string.find_last_of("_");
        std::string tmp_num_string = tmp_string.substr(npos+1,tmp_string.size() - type.size()-4);
        double tmp_num = atof(tmp_num_string.c_str());
        std::pair<double,std::string> tmp_pair;
        tmp_pair.first = tmp_num;
        tmp_pair.second = tmp_string;
        filelists_pair.push_back(tmp_pair);
    }
    std::sort(filelists_pair.begin(),filelists_pair.end(),computePairNum);
    filists.clear();
    for (int i = 0; i < filelists_pair.size(); ++i) {
        filists.push_back(filelists_pair[i].second);
    }
}

bool computeStrOrder(std::string str1, std::string str2)
{
    return str1 < str2;
}

void MainWindow::sort_filelists_kitti(std::vector<std::string>& filists)
{
    std::sort(filists.begin(),filists.end(),computeStrOrder);
}

void MainWindow::spinBox_nFrame_valueChanged(int){
    max_save_num_ = ui->spinBox_nFrame->value();
    publishString(pub_max_frame_num_,num2str(max_save_num_));
}

void MainWindow::on_pushButton_save_frame_clicked(){
    publishString(pub_frame_,"save_frame");
}

void MainWindow::on_pushButton_next_frame_clicked(){
    publishString(pub_frame_,"next_frame");
}

void MainWindow::on_pushButton_pre_frame_clicked(){
    publishString(pub_frame_,"pre_frame");
}

void MainWindow::on_pushButton_continue_clicked(){
    publishString(pub_continue_,"continue");
}

void MainWindow::on_pushButton_next_clicked(){
    publishString(pub_control_,"next");
}

/// ----  for PCD files -----

void MainWindow::setWidgetDisable(bool &is_file_empty)
{
    ui->comboBox_pcd->setDisabled(is_file_empty);
    ui->pushButton_continue_pcd->setDisabled(is_file_empty);
    ui->pushButton_save_index->setDisabled(is_file_empty);
    ui->pushButton_pre_pcd->setDisabled(is_file_empty);
    ui->pushButton_next_pcd->setDisabled(is_file_empty);
}

void MainWindow::publishString(const ros::Publisher& pub,const std::string& str){
    std_msgs::String tmp_msg;
    tmp_msg.data = str.c_str();
    pub.publish(tmp_msg);
}

void MainWindow::on_pushButton_load_pcd_clicked()
{
    QProPath_ = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "",
                                                  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    pcd_filelists_.clear();
    QString tmp_qstring;
    read_filelists(QProPath_.toStdString() + "/",pcd_filelists_,".pcd");
    if (pcd_filelists_.empty()){
        ui->textEdit_msg->append(tmp_qstring.fromStdString("No pcd files founded!!"));
        return;
    }

    ui->comboBox_pcd->clear();
    file_path_str_.clear();
    for (int i = 0; i < pcd_filelists_.size(); ++i) {
        ui->comboBox_pcd->addItem(tmp_qstring.fromStdString(pcd_filelists_[i]));
        file_path_str_ += (QProPath_.toStdString() + "/" + pcd_filelists_[i] + ";");
    }
    // pub to save node
    this->publishString( pub_pcd_con_signal_, "3" ); // load_sign = 3
    this->publishString( pub_pcd_info_, file_path_str_ );

    cur_pcd_index_ = 0;
    is_file_empty_ = false;
    this->setWidgetDisable( is_file_empty_ );
}

void MainWindow::on_pushButton_pre_pcd_clicked()
{
    if (pcd_filelists_.empty())
        return;

    publishString( pub_pcd_con_signal_, "1" );//stop_sign = 1
    publishString( pub_pcd_info_, "pre_pcd_signal" );
}

void MainWindow::on_pushButton_next_pcd_clicked()
{
    if (pcd_filelists_.empty())
        return;

    publishString( pub_pcd_con_signal_, "1" );//stop_sign = 1
    publishString( pub_pcd_info_, "next_pcd_signal" );
}

void MainWindow::on_pushButton_continue_pcd_clicked()
{
    if (pcd_filelists_.empty())
        return;

    publishString( pub_pcd_con_signal_, "2" );//continue_sign = 2
    publishString( pub_pcd_info_, "continue_pcd_signal" );
}

void MainWindow:: on_pushButton_save_index_clicked()
{
    if (pcd_filelists_.empty())
    {
        return;
        ui->textEdit_msg->append("Load pcd file first.");
    }

    publishString( pub_pcd_con_signal_, "1" );//stop_sign = 1
    publishString(pub_pcd_info_, "save_index");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::subCurIndex(std_msgs::String cur_index)
{
    int tmp_index = atoi(cur_index.data.c_str());
    if ( tmp_index >= 0 && tmp_index < pcd_filelists_.size() )
    {
        cur_pcd_index_ = tmp_index;
    }
    else
        ui->textEdit_msg->append( "Index out of range!" );
}


