#include <iostream>
#include <memory>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "endpoint_defines.h"
#include "marker_utils.h"
#include "route_optimization_strategy.h"
#include "route_optimizer.h"
#include "greedy_route_optimization_strategy.h"
#include "brute_force_route_optimization_strategy.h"

using namespace std;




RouteOptimizerNode::RouteOptimizerNode(const string& inputTopic, const string& vizTopic, const shared_ptr<IRouteOptimizationStrategy>& strategy):
 inputMarkersTopic_(inputTopic), visualizationTopic_(vizTopic), strategy_(strategy)
{
    sub_ = nh_.subscribe<visualization_msgs::MarkerArray>(inputMarkersTopic_, queueSize, &RouteOptimizerNode::optimalRouteVizulationCallback, this);
    pub_ = nh_.advertise<visualization_msgs::Marker>(visualizationTopic_, queueSize, true);
}


void RouteOptimizerNode::optimalRouteVizulationCallback(const visualization_msgs::MarkerArray::ConstPtr& markers){

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

    vector<geometry_msgs::Pose> optimizedRoute = strategy_->calculateOptimizedPath(markers);

    for(const auto& pose: optimizedRoute)
    {
        optimizedRouteMsg.points.push_back(pose.position);
    }
    pub_.publish(optimizedRouteMsg);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "route_optimizer");

    auto strategy = make_shared<GreedyStrategy>();
    // auto strategy = make_shared<BruteForceStrategy>();

    RouteOptimizerNode routeOptimizerNode(markerPointsTopic, optimizedRouteTopic, strategy);

    ros::spin();

    return 0;
}