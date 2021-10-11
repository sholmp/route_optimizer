#ifndef MARKER_UTILS_H
#define MARKER_UTILS_H

#include <vector>
#include <visualization_msgs/Marker.h>


std::vector<visualization_msgs::Marker> generateRandomPoints(int n, double gridWidth);

/**
 * will color markers in increasingly stronger green colors
 * first marker will be black
 */
void addPathShading(std::vector<visualization_msgs::Marker>& points);


#endif