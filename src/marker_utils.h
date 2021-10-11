#ifndef MARKER_UTILS_H
#define MARKER_UTILS_H

#include <vector>
#include <random>
#include <visualization_msgs/Marker.h>


std::vector<visualization_msgs::Marker> generateRandomPoints(int n, double gridWidth);

#endif