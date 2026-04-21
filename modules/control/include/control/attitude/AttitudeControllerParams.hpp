#pragma once

#include "math/Vector3.hpp"

namespace control::attitude
{

struct AttitudeControllerParams
{
   math::Vec3f gains_p{6.0f, 6.0f, 0.0f};
};

}   // namespace control::attitude
