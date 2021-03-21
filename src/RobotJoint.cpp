#include <RobotJoint.h>

// --- Filter Coefficients --- //
float RobotJoint::a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {1, 2.369513007182038, 2.313988414415880, 1.054665405878567, 0.187379492368185};
float RobotJoint::b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {0.432846644990292, 1.731386579961168, 2.597079869941751, 1.731386579961168, 0.432846644990292};

float RobotJoint::a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {0.816751544997935, 4.900509269987608, 12.251273174969020, 16.335030899958692, 12.251273174969020, 4.900509269987608, 0.816751544997935};
float RobotJoint::b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {1, 5.595430092802001, 13.058177871415467, 16.268238545772697, 11.410789023781913, 4.272380259839228, 0.667083086256514};

// --- Constructors --- //
RobotJoint::RobotJoint()
{
    currentFilter.setCoefficients(a_coefficients_currentFilter, b_coefficients_currentFilter);
    positonFilter.setCoefficients(a_coefficients_positionFilter, b_coefficients_positionFilter);
}
