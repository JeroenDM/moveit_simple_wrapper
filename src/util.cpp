#include "moveit_simple_wrapper/types.h"

namespace moveit_simple_wrapper
{
JointValues interpolate(JointValues q_from, JointValues q_to, double s)
{
    JointValues q(q_from.size());
    for (std::size_t i = 0; i < q_from.size(); ++i)
    {
        q[i] = (1 - s) * q_from[i] + s * q_to[i];
    }
    return q;
}
}