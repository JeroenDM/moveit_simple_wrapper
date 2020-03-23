#ifndef _MSRW_UTIL_H
#define _MSRW_UTIL_H

namespace moveit_simple_wrapper
{
JointValues interpolate(JointValues q_from, JointValues q_to, double s);
}
#endif