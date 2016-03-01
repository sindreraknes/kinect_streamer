/**
 * @file /include/kinect_streamer/main_window.hpp
 *
 * @brief Qt based gui for kinect_streamer.
 *
 * @date November 2010
 **/
#ifndef kinect_streamer_MAIN_WINDOW_H
#define kinect_streamer_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kinect_streamer {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
    void closeEvent(QCloseEvent *event);


public Q_SLOTS:
    void on_button_refresh1_clicked(bool check);
    void on_button_subscribe1_clicked(bool check);
    void on_button_refresh2_clicked(bool check);
    void on_button_subscribe2_clicked(bool check);
    void on_button_connect_clicked(bool check);
    void on_button_localhost_clicked(bool check);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace kinect_streamer

#endif // kinect_streamer_MAIN_WINDOW_H
