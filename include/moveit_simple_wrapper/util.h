/** Usefull functions.
 *
 * interpolation is used for collision checking along a path.
 * */
#ifndef _MSRW_UTIL_H
#define _MSRW_UTIL_H

namespace moveit_simple_wrapper
{
JointPositions interpolate(JointPositions q_from, JointPositions q_to, double s);
}
#endif