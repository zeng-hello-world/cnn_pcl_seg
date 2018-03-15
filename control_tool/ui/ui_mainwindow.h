/********************************************************************************
** Form generated from reading UI file 'mainwindowYG8892.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MAINWINDOWYG8892_H
#define MAINWINDOWYG8892_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTextBrowser *textEdit_msg;
    QTabWidget *tabWidget;
    QWidget *tab;
    QSpinBox *spinBox_nFrame;
    QPushButton *pushButton_pre_frame;
    QPushButton *pushButton_next_frame;
    QPushButton *pushButton_save_frame;
    QPushButton *pushButton_continue;
    QPushButton *pushButton_next;
    QWidget *tab_2;
    QPushButton *pushButton_load_pcd;
    QComboBox *comboBox_pcd;
    QPushButton *pushButton_save_index;
    QPushButton *pushButton_next_pcd;
    QPushButton *pushButton_pre_pcd;
    QPushButton *pushButton_continue_pcd;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(413, 230);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        textEdit_msg = new QTextBrowser(centralwidget);
        textEdit_msg->setObjectName(QStringLiteral("textEdit_msg"));
        textEdit_msg->setGeometry(QRect(10, 10, 391, 81));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(10, 100, 391, 121));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        spinBox_nFrame = new QSpinBox(tab);
        spinBox_nFrame->setObjectName(QStringLiteral("spinBox_nFrame"));
        spinBox_nFrame->setGeometry(QRect(10, 10, 71, 27));
        spinBox_nFrame->setMinimum(1);
        spinBox_nFrame->setSingleStep(1);
        spinBox_nFrame->setValue(10);
        pushButton_pre_frame = new QPushButton(tab);
        pushButton_pre_frame->setObjectName(QStringLiteral("pushButton_pre_frame"));
        pushButton_pre_frame->setGeometry(QRect(240, 10, 91, 27));
        pushButton_next_frame = new QPushButton(tab);
        pushButton_next_frame->setObjectName(QStringLiteral("pushButton_next_frame"));
        pushButton_next_frame->setGeometry(QRect(120, 10, 91, 27));
        pushButton_save_frame = new QPushButton(tab);
        pushButton_save_frame->setObjectName(QStringLiteral("pushButton_save_frame"));
        pushButton_save_frame->setGeometry(QRect(110, 40, 71, 41));
        pushButton_continue = new QPushButton(tab);
        pushButton_continue->setObjectName(QStringLiteral("pushButton_continue"));
        pushButton_continue->setGeometry(QRect(210, 40, 81, 41));
        pushButton_next = new QPushButton(tab);
        pushButton_next->setObjectName(QStringLiteral("pushButton_next"));
        pushButton_next->setGeometry(QRect(300, 40, 81, 41));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        pushButton_load_pcd = new QPushButton(tab_2);
        pushButton_load_pcd->setObjectName(QStringLiteral("pushButton_load_pcd"));
        pushButton_load_pcd->setGeometry(QRect(10, 10, 85, 27));
        comboBox_pcd = new QComboBox(tab_2);
        comboBox_pcd->setObjectName(QStringLiteral("comboBox_pcd"));
        comboBox_pcd->setGeometry(QRect(100, 10, 281, 27));
        pushButton_save_index = new QPushButton(tab_2);
        pushButton_save_index->setObjectName(QStringLiteral("pushButton_save_index"));
        pushButton_save_index->setGeometry(QRect(10, 40, 81, 41));
        pushButton_next_pcd = new QPushButton(tab_2);
        pushButton_next_pcd->setObjectName(QStringLiteral("pushButton_next_pcd"));
        pushButton_next_pcd->setGeometry(QRect(290, 40, 85, 41));
        pushButton_pre_pcd = new QPushButton(tab_2);
        pushButton_pre_pcd->setObjectName(QStringLiteral("pushButton_pre_pcd"));
        pushButton_pre_pcd->setGeometry(QRect(200, 40, 85, 41));
        pushButton_continue_pcd = new QPushButton(tab_2);
        pushButton_continue_pcd->setObjectName(QStringLiteral("pushButton_continue_pcd"));
        pushButton_continue_pcd->setGeometry(QRect(110, 40, 81, 41));
        tabWidget->addTab(tab_2, QString());
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Auto Play", 0));
        pushButton_pre_frame->setText(QApplication::translate("MainWindow", "pre frame", 0));
        pushButton_next_frame->setText(QApplication::translate("MainWindow", "next frame", 0));
        pushButton_save_frame->setText(QApplication::translate("MainWindow", "save", 0));
        pushButton_continue->setText(QApplication::translate("MainWindow", "continue", 0));
        pushButton_next->setText(QApplication::translate("MainWindow", "next", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "pcb", 0));
        pushButton_load_pcd->setText(QApplication::translate("MainWindow", "load", 0));
        pushButton_save_index->setText(QApplication::translate("MainWindow", "save index", 0));
        pushButton_next_pcd->setText(QApplication::translate("MainWindow", "next ->", 0));
        pushButton_pre_pcd->setText(QApplication::translate("MainWindow", "<- pre", 0));
        pushButton_continue_pcd->setText(QApplication::translate("MainWindow", "continue", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "pcd", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINWINDOWYG8892_H
