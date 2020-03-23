#ifndef _MSRW_TYPES_H
#define _MSRW_TYPES_H

#include <vector>
#include <Eigen/Geometry>

namespace moveit_simple_wrapper
{

typedef Eigen::Isometry3d Transform;
typedef std::vector<std::vector<double>> IKSolution;
typedef std::vector<double> JointValues;

} // namespace moveit_simple_wrapper
#endif