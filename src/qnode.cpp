/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/kinect_streamer/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kinect_streamer {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"kinect_streamer");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    pointCloudPub = n.advertise<CustomPointCloud>("PointClouds", 10);
    cloud0New = false;
    cloud1New = false;
    // Add your ros communications here.
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"kinect_streamer");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    pointCloudPub = n.advertise<CustomPointCloud>("PointClouds", 10);
    cloud0New = false;
    cloud1New = false;
    // Add your ros communications here.
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(200);
	while ( ros::ok() ) {

        if(cloud0New && cloud1New){
            CustomPointCloud msg;
            msg.cloud1 = cloud0;
            msg.cloud2 = cloud0;
            pointCloudPub.publish(msg);
            cloud0New = false;
            cloud1New = false;
        }

		ros::spinOnce();
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

QStringList QNode::getTopics()
{
    QStringList list;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/PointCloud2", Qt::CaseInsensitive) == 0){
            list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    return list;
}

void QNode::subscribeToPointCloud2(QString topic, int kinectNr)
{
    ros::NodeHandle n;
    const char *tmp = topic.toUtf8().constData();
    switch(kinectNr){
    case 0:
        pointCloud2Sub0 = n.subscribe<sensor_msgs::PointCloud2, QNode>(tmp, 1, &QNode::cloudCallback0, this);
        break;
    case 1:
        pointCloud2Sub1 = n.subscribe<sensor_msgs::PointCloud2, QNode>(tmp, 1, &QNode::cloudCallback1, this);
        break;
    }
}

void QNode::cloudCallback0(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    cloud0 = *cloud_msg;
    cloud0New = true;
}

void QNode::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    cloud1 = *cloud_msg;
    cloud1New = true;
}


}  // namespace kinect_streamer
