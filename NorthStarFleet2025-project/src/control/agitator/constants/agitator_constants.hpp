#ifndef AGITATOR_CONSTANTS_HPP_
#define AGITATOR_CONSTANTS_HPP_

#ifdef STANDARD_CONSTANTS
#include "robot/standard/standard_agitator_constants.hpp"
#elif SENTRY_CONSTANTS
#include "robot/sentry/sentry_agitator_constants.hpp"
#elif HERO_CONSTANTS
#include "robot/hero/hero_agitator_constants.hpp"
#elif TURRET
#include "robot/standard/standard_agitator_constants.hpp"
#else
#include "robot/standard/standard_agitator_constants.hpp"
#endif

#endif  // AGITATOR_CONSTANTS_HPP_
