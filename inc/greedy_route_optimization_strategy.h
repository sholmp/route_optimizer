#ifndef GREEDY_ROUTE_OPTIMIZATION_STRATEGY_H
#define GREEDY_ROUTE_OPTIMIZATION_STRATEGY_H

#include "route_optimization_strategy.h"

class GreedyStrategy : public IRouteOptimizationStrategy
{
public:
    GreedyStrategy() = default;

    std::vector<geometry_msgs::Pose> calculateOptimizedPath(const visualization_msgs::MarkerArray::ConstPtr& markers) override;

};

#endif