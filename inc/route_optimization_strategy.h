#ifndef ROUTE_OPTIMIZATION_STRATEGY_H
#define ROUTE_OPTIMIZATION_STRATEGY_H

#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

/**
 * Route optimization interface
 * A concrete implementation must take a MarkerArray and return
 * a set of points ordered such that an optimized path is created through all points
 */
class IRouteOptimizationStrategy
{
public:
    virtual ~IRouteOptimizationStrategy(){}

    virtual std::vector<geometry_msgs::Pose> calculateOptimizedPath(const visualization_msgs::MarkerArray::ConstPtr & markers) = 0;

};

#endif