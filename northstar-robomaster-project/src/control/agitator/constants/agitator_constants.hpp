#ifndef AGITATOR_CONSTANTS_HPP_
#define AGITATOR_CONSTANTS_HPP_

#ifdef TARGET_STANDARD
#include "robot/standard/standard_agitator_constants.hpp"
#elif TARGET_SENTRY
#include "robot/sentry/sentry_agitator_constants.hpp"
#elif TARGET_HERO
#include "robot/hero/hero_agitator_constants.hpp"
#elif TURRET
#include "robot/standard/standard_agitator_constants.hpp"
#elif TARGET_TEST_BED
#include "robot/testbed/testbed_agitator_constants.hpp"
#else
#include "robot/standard/standard_agitator_constants.hpp"
#endif

#endif  // AGITATOR_CONSTANTS_HPP_
