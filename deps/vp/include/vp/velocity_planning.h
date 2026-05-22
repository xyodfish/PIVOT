/**
 * @file velocity_planning.h
 * @brief Velocity Planning Library - Main Include Header
 * 
 * Include this header to access all velocity planning functionality.
 * 
 * @example
 * #include <vp/velocity_planning.h>
 */

#ifndef VP_VELOCITY_PLANNING_H
#define VP_VELOCITY_PLANNING_H

// Core interfaces
#include "velocity_planner_interface.h"
#include "planner_exception.h"

// Planners
#include "trapezoidal_planner.h"
#include "double_s_planner.h"
#include "multi_velocity_planner.h"

// Geometry trajectory
#include "geometry_trajectory/straight_trajectory.h"

#endif  // VP_VELOCITY_PLANNING_H
