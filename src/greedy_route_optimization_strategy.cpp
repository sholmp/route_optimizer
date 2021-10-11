#include "greedy_route_optimization_strategy.h"
#include "geometry_msgs/Pose.h"
#include "marker_utils.h"
#include "visualization_msgs/Marker.h"

using namespace std;

vector<geometry_msgs::Pose> GreedyStrategy::calculateOptimizedPath(const visualization_msgs::MarkerArray::ConstPtr& markers){
    
    vector<geometry_msgs::Pose> optimizedRoute;
    vector<visualization_msgs::Marker> markersCopy = markers->markers;

    visualization_msgs::Marker start;
    start.pose.position.x = 0;
    start.pose.position.y = 0;
    start.pose.position.z = 0;

    auto comparator = [&start](const visualization_msgs::Marker& m1, const visualization_msgs::Marker& m2){
        return euclideanDistBetweenMarkers(m1, start) > euclideanDistBetweenMarkers(m2, start);
    };

    while(markersCopy.size())
    {
        sort(markersCopy.begin(), markersCopy.end(), comparator);

        start = markersCopy.back();
        optimizedRoute.push_back(start.pose);
        markersCopy.pop_back();
    }

    cout << "\nOptimized route:" << endl;
    for(auto point : optimizedRoute)
        cout << point << endl;

    return optimizedRoute;
}
