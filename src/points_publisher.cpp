#include <iostream>
#include <vector>
#include <random>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "endpoint_defines.h"
#include "marker_utils.h"

using namespace std;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "points_publisher");

    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(markerPointsTopic, 10, true); // latch the message so it will stay in topic queue

    int numberOfRandomPoints = 10;
    int gridWidth = 5;
    vector<visualization_msgs::Marker> markers = generateRandomPoints(numberOfRandomPoints, gridWidth);
    visualization_msgs::MarkerArray markersMsg;
    markersMsg.markers = markers;


    pub.publish(markersMsg);

    sleep(1); // Don't terminate process immediately after pub (otherwise recipients are not notified)

    return 0;
}