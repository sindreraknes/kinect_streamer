/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/kinect_streamer/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kinect_streamer {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


}

MainWindow::~MainWindow() {}


void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::on_button_refresh1_clicked(bool check)
{
    ui.comboBox->addItems(qnode.getTopics());
    ui.comboBox_2->addItems(qnode.getTopics());
}

void MainWindow::on_button_subscribe1_clicked(bool check)
{
    qnode.subscribeToPointCloud2(ui.comboBox->currentText(), 0);
}

void MainWindow::on_button_refresh2_clicked(bool check)
{
    ui.comboBox->addItems(qnode.getTopics());
    ui.comboBox_2->addItems(qnode.getTopics());
}

void MainWindow::on_button_subscribe2_clicked(bool check)
{
    qnode.subscribeToPointCloud2(ui.comboBox_2->currentText(), 1);
}

void MainWindow::on_button_connect_clicked(bool check)
{
    qnode.init(ui.line_edit_master->text().toStdString(), ui.line_edit_host->text().toStdString());
}

void MainWindow::on_button_localhost_clicked(bool check)
{
    qnode.init();
}

}  // namespace kinect_streamer

