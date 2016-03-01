/**
 * @file /include/kinect_streamer/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef kinect_streamer_QNODE_HPP_
#define kinect_streamer_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/PointCloud2.h>
#include "kinect_streamer/CustomPointCloud.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kinect_streamer {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    QStringList getTopics();
    void subscribeToPointCloud2(QString topic, int kinectNr);

    void cloudCallback0(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);



Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
    ros::Publisher pointCloudPub;
    ros::Subscriber pointCloud2Sub0;
    ros::Subscriber pointCloud2Sub1;
    sensor_msgs::PointCloud2 cloud0;
    sensor_msgs::PointCloud2 cloud1;
    bool cloud0New;
    bool cloud1New;
};

}  // namespace kinect_streamer

#endif /* kinect_streamer_QNODE_HPP_ */
