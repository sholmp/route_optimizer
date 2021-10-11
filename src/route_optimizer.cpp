#include <iostream>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "endpoint_defines.h"
#include "marker_utils.h"

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

ostream& operator<<(ostream& lhs, const visualization_msgs::Marker& marker)
{
    lhs << "(" << marker.pose.position.x << ", "
     << marker.pose.position.y << ", " 
     << marker.pose.position.z << ")";
     return lhs;
}

void markerPointsCallback(const visualization_msgs::MarkerArray::ConstPtr& points)
{

    vector<visualization_msgs::MarkerArray> optmizedRouteMsg;
    vector<visualization_msgs::Marker> optimizedRoute;

    // vector<visualization_msgs::Marker> pointsCopy = points->markers;
    vector<visualization_msgs::Marker> pointsCopy = generateRandomPoints(10, 5);

    // seed node should be closest to (0,0)
    // for now, just first element of points:


    


    // vector<bool> visited(points->markers.size());
    // visited[0] = true;
    // for(int i = 0; i < points->markers.size(); i++)
    // {
    //     if(!visited[i])
    //     {
            
    //     }
    // }

    // visualization_msgs::Marker start;
    // start.pose.position = 


    //create a dummy entry point (0,0,0)
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

    

    // cout << "after sort!" << endl;
    // for(auto point : pointsCopy)
    //     cout << point << endl;
    cout << "\nOptimized route:" << endl;
    for(auto point : optimizedRoute)
        cout << point << endl;

}


// void markerPoints

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "route_optimizer");

    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>(markerPointsTopic, 10, markerPointsCallback);

    pub = nh.advertise<visualization_msgs::MarkerArray>(optimizedRouteTopic, 10, true);

    visualization_msgs::MarkerArray msg;

    // markerPointsCallback(&msg);

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