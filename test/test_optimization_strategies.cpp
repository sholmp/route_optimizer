#include <iostream>
#include <memory>
#include <gtest/gtest.h>

#include "brute_force_route_optimization_strategy.h"
#include "greedy_route_optimization_strategy.h"
#include "marker_utils.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;


TEST(StrategySuite, DistanceComparisonTest){
    
    BruteForceStrategy brute;
    GreedyStrategy greedy;

    vector<visualization_msgs::Marker> markers = generateRandomPoints(6, 5);


    visualization_msgs::MarkerArray msg;
    msg.markers = markers;
    
    visualization_msgs::MarkerArray::ConstPtr ptr (new visualization_msgs::MarkerArray(msg));

    vector<geometry_msgs::Pose> pathBrute =  brute.calculateOptimizedPath(ptr);
    vector<geometry_msgs::Pose> pathGreedy = greedy.calculateOptimizedPath(ptr);

    auto calcDist = [](vector<geometry_msgs::Pose>& poses){
        double totalDist = 0;
        for(int i = 0; i < poses.size() - 1; i++)
            totalDist += euclideanDistBetweenPoses(poses[i], poses[i + 1]);
        return totalDist;
    };

    // The bruteforce path should always be shorter or at least equally as short as the greedy

    EXPECT_LE(calcDist(pathBrute), calcDist(pathGreedy)); 
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}