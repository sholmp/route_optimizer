#ifndef BRUTE_FORCE_OPTIMIZATION_STRATEGY_H
#define BRUTE_FORCE_OPTIMIZATION_STRATEGY_H

#include "route_optimization_strategy.h"

class BruteForceStrategy : public IRouteOptimizationStrategy
{
public:
    BruteForceStrategy() = default;

    std::vector<geometry_msgs::Pose> calculateOptimizedPath(const visualization_msgs::MarkerArray::ConstPtr & markers) override;

};

#endif