#ifndef MARKER_UTILS_H
#define MARKER_UTILS_H

#include <vector>
#include <ostream>
#include <visualization_msgs/Marker.h>


std::vector<visualization_msgs::Marker> generateRandomPoints(int n, double gridWidth);

/**
 * will color markers in increasingly stronger green colors
 * first marker will be black
 */
void addPathShading(std::vector<visualization_msgs::Marker>& points);


/*
 * calculates l2 norm between @param m1 and @param m2
 */
double euclideanDistBetweenMarkers(const visualization_msgs::Marker& m1, const::visualization_msgs::Marker& m2);

/**
 * Format a Marker and send to output stream
 */
std::ostream& operator<<(std::ostream& lhs, const visualization_msgs::Marker& marker);


#endif