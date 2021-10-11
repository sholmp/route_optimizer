#include <algorithm>
#include <limits>
#include <algorithm>
#include <numeric>

#include "brute_force_route_optimization_strategy.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "marker_utils.h"
#include <ros/console.h>

using namespace std;

vector<geometry_msgs::Pose> BruteForceStrategy::calculateOptimizedPath(const visualization_msgs::MarkerArray::ConstPtr& markers){

    vector<visualization_msgs::Marker> markersCopy = markers->markers;
    vector<visualization_msgs::Marker> shortestRoute;

    vector<int> indicies(markersCopy.size());
    iota(indicies.begin(), indicies.end(), 0); // create increasing sequence [0,1,2,...]
    vector<int> shortestIndicies(indicies.size()); // Stores the sequence indicies producing the shortest path


    double minDistance = numeric_limits<double>::max();
    do{
        double dist = totalDistanceFollowingIndicies(markersCopy, indicies);
        if(dist < minDistance)
        {
            minDistance = dist;
            copy(indicies.begin(), indicies.end(), shortestIndicies.begin());
        }
    }while(next_permutation(indicies.begin(), indicies.end()));

    // Debugging:
    // for(auto marker : markersCopy)
    //     cout << marker << endl;

    // cout << "-------------------\n\n";

    // for(int idx : shortestIndicies)
    //     cout << idx << ", ";
    // cout << endl;

    // cout << "-------------------------\n";

    vector<geometry_msgs::Pose> optimizedRoute;
    for(int idx = 0; idx < shortestIndicies.size(); idx++)
    {
        cout << markersCopy[shortestIndicies[idx]] << endl;
        optimizedRoute.push_back(markersCopy[shortestIndicies[idx]].pose);
    }

    return optimizedRoute;
}
