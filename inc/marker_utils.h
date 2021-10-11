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
 * calculates the euclidean distance it takes to travel from @param markers[0] -> markers[size-1]
 * visiting every node in the order of the vector
 */
double totalInOrderDistance(const std::vector<visualization_msgs::Marker>& markers);


/**
 * calculates the euclidean distance it takes to travel through @param markers in sequence
 * specified by @param indiciesSequence
 */
double totalDistanceFollowingIndicies(const std::vector<visualization_msgs::Marker>& markers, const std::vector<int>& indiciesSequence);

/**
 * Format a Marker and send to output stream
 */
std::ostream& operator<<(std::ostream& lhs, const visualization_msgs::Marker& marker);


#endif