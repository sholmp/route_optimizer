#ifndef ROUTE_OPTIMIZER_H
#define ROUTE_OPTIMIZER_H

#include <string>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

class RouteOptimizerNode
{
public:
    RouteOptimizerNode(const std::string& inputTopic, const std::string& vizTopic);

    /**
    *  Calculate an optimized route through @points using greedy path planning.
    *  Will assume a starting point of (0,0,0)
    */
    void optimalRouteVizulationCallback(const visualization_msgs::MarkerArray::ConstPtr& markers);
        

private:
    std::string inputMarkersTopic_;
    std::string visualizationTopic_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    static inline int queueSize = 10;

};


#endif