/** All typedefs in one place.
 * This allows us to change for example the transform type
 * in a single place for all the code.
 * Or I could potentially use floats instead of doubles by changing this file.
 * */
#ifndef _MSRW_TYPES_H
#define _MSRW_TYPES_H

#include <vector>
#include <Eigen/Geometry>

namespace moveit_simple_wrapper
{
typedef Eigen::Isometry3d Transform;
typedef std::vector<std::vector<double>> IKSolution;
typedef std::vector<double> JointPositions;

}  // namespace moveit_simple_wrapper
#endif