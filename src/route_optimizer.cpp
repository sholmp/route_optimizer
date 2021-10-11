#include <iostream>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "endpoint_defines.h"
#include "marker_utils.h"

using namespace std;

/*
 * calculates l2 norm between @param m1 and @param m2
 */
double euclideanDistBetweenMarkers(const visualization_msgs::Marker& m1, const::visualization_msgs::Marker& m2)
{

    double squaredXdiff = pow(m1.pose.position.x - m2.pose.position.x, 2);
    double squaredYdiff = pow(m1.pose.position.y - m2.pose.position.y, 2);
    double squaredZdiff = pow(m1.pose.position.z - m2.pose.position.z, 2);
    
    return sqrt(squaredXdiff + squaredYdiff + squaredZdiff);
}

ros::Publisher pub;

ostream& operator<<(ostream& lhs, const visualization_msgs::Marker& marker)
{
    lhs << "(" << marker.pose.position.x << ", "
     << marker.pose.position.y << ", " 
     << marker.pose.position.z << ")";
     return lhs;
}

/**
 *  Calculate an optimized route through @points using greedy path planning.
 *  Will assume a starting point of (0,0,0)
 */
void markerPointsCallback(const visualization_msgs::MarkerArray::ConstPtr& points)
{
    vector<visualization_msgs::Marker> optimizedRoute;
    vector<visualization_msgs::Marker> pointsCopy = points->markers;

    // seed node chosen to be closest to (0,0)
    visualization_msgs::Marker start;
    start.pose.position.x = 0;
    start.pose.position.y = 0;
    start.pose.position.z = 0;

    cout << "points in random order:" << endl;
    for(auto point: pointsCopy)
        cout << point << endl;

    auto comparator = [&start](const visualization_msgs::Marker& m1, const visualization_msgs::Marker& m2){
        return euclideanDistBetweenMarkers(m1, start) > euclideanDistBetweenMarkers(m2, start);
    };

    while(pointsCopy.size())
    {
        sort(pointsCopy.begin(), pointsCopy.end(), comparator);

        start = pointsCopy.back();
        optimizedRoute.push_back(start);
        pointsCopy.pop_back();
    }

    cout << "\nOptimized route:" << endl;
    for(auto point : optimizedRoute)
        cout << point << endl;

    // Insert boilerplate attributes for visualization purposes:

    visualization_msgs::Marker optimizedRouteMsg;
    optimizedRouteMsg.header.stamp = ros::Time();
    optimizedRouteMsg.header.frame_id = "map";
    optimizedRouteMsg.ns = "my_namespace";
    optimizedRouteMsg.id = 0;
    optimizedRouteMsg.type = visualization_msgs::Marker::LINE_STRIP;
    optimizedRouteMsg.action = visualization_msgs::Marker::ADD; 
    optimizedRouteMsg.pose.orientation.w = 1.0;
    optimizedRouteMsg.scale.x = 0.05;
    optimizedRouteMsg.scale.y = 0.05;
    optimizedRouteMsg.scale.z = 0.05;
    optimizedRouteMsg.color.a = 1.0; // Don't forget to set the alpha!
    optimizedRouteMsg.color.r = 0.0;
    optimizedRouteMsg.color.g = 1;
    optimizedRouteMsg.color.b = 0.0;

    for(const auto& p: optimizedRoute)
    {
        optimizedRouteMsg.points.push_back(p.pose.position);
    }
    pub.publish(optimizedRouteMsg);

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "route_optimizer");

    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>(markerPointsTopic, 10, markerPointsCallback);

    pub = nh.advertise<visualization_msgs::Marker>(optimizedRouteTopic, 10, true);

    ros::spin();


    return 0;
}