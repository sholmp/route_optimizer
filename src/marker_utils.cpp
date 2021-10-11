#include <random>

#include "marker_utils.h"


using namespace std;

/**
 * generate @param n random markers with position [0 - @param gridWidth] in x,y. 
 * z is 0 for all points
 */
vector<visualization_msgs::Marker> generateRandomPoints(int n, double gridWidth)
{

    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0,gridWidth);

    vector<visualization_msgs::Marker> points;

    for(int i = 0; i < n; i++)
    {

        visualization_msgs::Marker marker;

        // boiler plate
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "map";
        marker.ns = "my_namespace"; // ???
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE; // POINTS
        marker.action = visualization_msgs::Marker::ADD; // ???

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // generate random positions:
        marker.pose.position.x = dis(gen);
        marker.pose.position.y = dis(gen);
        marker.pose.position.z = 0;

        points.push_back(marker);
    }

    return points;
}