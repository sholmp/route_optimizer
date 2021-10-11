#include <iostream>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "endpoint_defines.h"

using namespace std;


/*
 * 
 */
double euclideanDistBetweenMarkers(const visualization_msgs::Marker& m1, const::visualization_msgs::Marker& m2)
{

    double squaredXdiff = pow(m1.pose.position.x - m2.pose.position.x, 2);
    double squaredYdiff = pow(m1.pose.position.y - m2.pose.position.y, 2);
    double squaredZdiff = pow(m1.pose.position.z - m2.pose.position.z, 2);
    
    return sqrt(squaredXdiff + squaredYdiff + squaredZdiff);
}

ros::Publisher pub;

// static ros::NodeHandle nh;
void markerPointsCallback(const visualization_msgs::MarkerArray::ConstPtr& points)
{

    vector<visualization_msgs::MarkerArray> optimizedRoute;

    // seed node should be closest to (0,0)
    // for now, just first element of points:

    visualization_msgs::Marker start = points->markers[0];
    vector<bool> visited(points->markers.size());
    visited[0] = true;
    for(int i = 0; i < points->markers.size(); i++)
    {
        if(!visited[i])
        {
            
        }
    }

    // visualization_msgs::Marker start;
    // start.pose.position = 


    // auto comparator = [&start](const visualization_msgs::Marker& m1, const visualization_msgs::Marker& m2){
    //     return euclideanDistBetweenMarkers(m1, start) < euclideanDistBetweenMarkers(m2, start);
    // };
    // sort(points->markers.begin(), points->markers.end(), comparator);




    ROS_INFO("I was called!");
}


// void markerPoints

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "route_optimizer");

    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>(markerPointsTopic, 10, markerPointsCallback);

    pub = nh.advertise<visualization_msgs::MarkerArray>(optimizedRouteTopic, 10, true);

    
    // visualization_msgs::Marker m1;
    // m1.pose.position.x = 3;
    // m1.pose.position.y = 4;
    // m1.pose.position.z = 0;

    // visualization_msgs::MarkerArray msg;
    // msg.markers = {m1};

    // pub.publish(msg);

    ros::spin();



    // visualization_msgs::Marker m2;

    // m2.pose.position.x = 0;
    // m2.pose.position.y = 0;
    // m2.pose.position.z = 0;

    // cout << euclideanDistBetweenMarkers(m1, m2);


    return 0;
}