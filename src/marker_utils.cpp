#include <random>
#include <stdexcept>

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
        marker.type = visualization_msgs::Marker::SPHERE; 
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

double euclideanDistBetweenMarkers(const visualization_msgs::Marker& m1, const visualization_msgs::Marker& m2)
{

    double squaredXdiff = pow(m1.pose.position.x - m2.pose.position.x, 2);
    double squaredYdiff = pow(m1.pose.position.y - m2.pose.position.y, 2);
    double squaredZdiff = pow(m1.pose.position.z - m2.pose.position.z, 2);
    
    return sqrt(squaredXdiff + squaredYdiff + squaredZdiff);
}

double euclideanDistBetweenPoses(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    double squaredXdiff = pow(p1.position.x - p2.position.x, 2);
    double squaredYdiff = pow(p1.position.y - p2.position.y, 2);
    double squaredZdiff = pow(p1.position.z - p2.position.z, 2);
    
    return sqrt(squaredXdiff + squaredYdiff + squaredZdiff);
}


double totalInOrderDistance(const std::vector<visualization_msgs::Marker>& markers){
    double total = 0;

    for(int i = 0; i < markers.size() - 1; i++){
        total += euclideanDistBetweenMarkers(markers[i], markers[i + 1]);
    }

    return total;
}

double totalDistanceFollowingIndicies(const std::vector<visualization_msgs::Marker>& markers, const std::vector<int>& indiciesSequence){
    double total = 0;

    try{
        for(int i = 0; i < indiciesSequence.size() - 1; i++)
        {
            total += euclideanDistBetweenMarkers(markers[indiciesSequence[i]], markers[indiciesSequence[i + 1]]);
        }
    }
    catch(std::out_of_range& e)
    {
        cout << "bad indiciesSequence" << endl;
        return -1.0;
    }

    return total;
}


ostream& operator<<(ostream& lhs, const visualization_msgs::Marker& marker)
{
    lhs << "(" << marker.pose.position.x << ", "
     << marker.pose.position.y << ", " 
     << marker.pose.position.z << ")";
     return lhs;
}

void addPathShading(vector<visualization_msgs::Marker>& points)
{
    int i = 0;
    for(auto& marker : points)
    {
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "map";
        marker.ns = "my_namespace";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD; 

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1 - i / (double)points.size();
        marker.color.g = i / (double)points.size();
        marker.color.b = 0.0;

        i++; // used for marker.id (not sure if necessary)
    }
}
