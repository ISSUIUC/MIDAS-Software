#    ----------------------------------
# SAFETY THRESHOLDS
# ----------------------------------

# Transition back to STATE_SAFE if this much time has passed without firing a pyro (ms)
SAFETY_PYRO_TEST_DISARM_TIME = 10000

# ----------------------------------
# SECOND STAGE THRESHOLDS
# ----------------------------------

# Regardless of sensor inputs, stay on pyro firing states for at LEAST this time. (ms)
SUSTAINER_PYRO_FIRING_TIME_MINIMUM = 100

# Transition to FIRST_BOOST if acceleration is greater than this (G)
SUSTAINER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD = 5

# Return state to IDLE if not boosting for this amount of time (ms)
SUSTAINER_IDLE_TO_FIRST_BOOST_TIME_THRESHOLD = 1000

# Transition to SECOND_BOOST from SUSTAINER_IGNITION if acceleration greater than this (G)
SUSTAINER_IGNITION_TO_SECOND_BOOST_ACCELERATION_THRESHOLD = 8

# Return state to SECOND_BOOST if not boosting for this amount of time (ms)
SUSTAINER_SECOND_BOOST_TO_COAST_TIME_THRESHOLD = 1000

# Transition to COAST if acceleration is less than this value (g)
SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD = 0.2

# Reach apogee state when vertical speed is less than or equal to this value (m/s)
SUSTAINER_COAST_TO_APOGEE_VERTICAL_SPEED_THRESHOLD = 25

# Revert back to COAST if the vertical speed in apogee is too high (m/s)
SUSTAINER_APOGEE_BACKTO_COAST_VERTICAL_SPEED_THRESHOLD = 25

# Revert back to COAST if apogee was too brief (ms)
SUSTAINER_APOGEE_CHECK_THRESHOLD = 500

# Move on to DROGUE_DEPLOY after being in apogee for this amount of time (ms)
SUSTAINER_APOGEE_TIMER_THRESHOLD = 500

# Move on to DROGUE after a second of reaching apogee (ms)
SUSTAINER_DROGUE_TIMER_THRESHOLD = 3000

# Move on to MAIN after passing this amount of time (ms)
SUSTAINER_MAIN_TO_MAIN_DEPLOY_TIMER_THRESHOLD = 3000

# Height required to deploy the main parachutes (m)
SUSTAINER_MAIN_DEPLOY_ALTITUDE_THRESHOLD = 545

# The minimum delay between drogue deployment and main deployment (ms)
SUSTAINER_MAIN_DEPLOY_DELAY_AFTER_DROGUE = 1000

# Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
SUSTAINER_IGNITION_TO_SECOND_BOOST_TIME_THRESHOLD = 1000

# Transition straight to coast after a certain amount of time not detecting second stage boost (ms)
SUSTAINER_IGNITION_TO_COAST_TIMER_THRESHOLD = 5000

# Revert back to main if the landed was too short (ms)
SUSTAINER_LANDED_TIMER_THRESHOLD = 5000

# Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
SUSTAINER_COAST_TIME = 3000

# Transition to LANDED from MAIN if vertical speed is less than this threshold (m/s)
SUSTAINER_LANDED_VERTICAL_SPEED_THRESHOLD = 1

# Transition to MAIN from LANDED if vertical speed is greater than this threshold (m/s)
SUSTAINER_LANDED_TO_MAIN_VERTICAL_SPEED_THRESHOLD = 3

# Lock out further transitions from LANDED after this much time passes in the LANDED state (ms)
SUSTAINER_LANDED_TIME_LOCKOUT = 60000

# Prevent us from inadvertently entering the LANDED state when we're at a low velocity at main deploy (ms)
SUSTAINER_MAIN_TO_LANDED_LOCKOUT = 5000

# Stores a small jerk value (m/s^3)
SUSTAINER_DROGUE_JERK_THRESHOLD = 200

# Stores a small jerk value (m/s^3)
SUSTAINER_MAIN_JERK_THRESHOLD = 300

# ----------------------------------
# FIRST STAGE THRESHOLDS
# ----------------------------------

# Regardless of sensor inputs, stay on pyro firing states for at LEAST this time (ms)
BOOSTER_PYRO_FIRING_TIME_MINIMUM = 100

# Transition to FIRST_BOOST if acceleration is greater than this (G)
BOOSTER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD = 5

# Return state to IDLE if not boosting for this amount of time (ms)
BOOSTER_IDLE_TO_FIRST_BOOST_TIME_THRESHOLD = 1000

# Move on regardless if it separates or not i.e. if state is FIRST_SEPARATION for over this amount of time (ms)
BOOSTER_FIRST_SEPARATION_TIME_THRESHOLD = 3000

# Transition to COAST if acceleration is less than this value (g)
BOOSTER_COAST_DETECTION_ACCELERATION_THRESHOLD = 0.2

# Reach apogee state when vertical speed is less than or equal to this value (m/s)
BOOSTER_COAST_TO_APOGEE_VERTICAL_SPEED_THRESHOLD = 20

# Revert back to COAST if apogee was too brief (ms)
BOOSTER_APOGEE_CHECK_THRESHOLD = 500

# Move on to DROGUE_DEPLOY after being in apogee for this amount of time (ms)
BOOSTER_APOGEE_TIMER_THRESHOLD = 500

# Move on to DROGUE after a second of reaching apogee (ms)
BOOSTER_DROGUE_TIMER_THRESHOLD = 3000

# Move on to MAIN after passing this amount of time (ms)
BOOSTER_MAIN_TO_MAIN_DEPLOY_TIMER_THRESHOLD = 3000

# Height required to deploy the main parachutes (m) - Booster does not have a drogue
BOOSTER_MAIN_DEPLOY_ALTITUDE_THRESHOLD = 545

# The minimum delay between drogue deployment and main deployment (ms)
BOOSTER_MAIN_DEPLOY_DELAY_AFTER_DROGUE = 1000

# Return to SUSTAINER_IGNITION if not in SECOND_BOOST for this amount of time (ms)
BOOSTER_IGNITION_TO_SECOND_BOOST_TIME_THRESHOLD = 1000

# Transition straight to coast after a certain amount of time not detecting second stage boost (ms)
BOOSTER_IGNITION_TO_COAST_TIMER_THRESHOLD = 5000

# Revert back to main if the landed was too short (ms)
BOOSTER_LANDED_TIMER_THRESHOLD = 5000

# Return state to FIRST_BOOST if not in BURNOUT for this amount of time (ms)
BOOSTER_FIRST_BOOST_TO_BURNOUT_TIME_THRESHOLD = 1000

# Transition to LANDED from MAIN if vertical speed is less than this threshold (m/s)
BOOSTER_LANDED_VERTICAL_SPEED_THRESHOLD = 1

# Transition to MAIN from LANDED if vertical speed is greater than this threshokd (m/s)
BOOSTER_LANDED_TO_MAIN_VERTICAL_SPEED_THRESHOLD = 5

# Lock out further transitions from LANDED after this much time passes in the LANDED state (ms)
BOOSTER_LANDED_TIME_LOCKOUT = 60000

# Prevent us from inadvertently entering the LANDED state when we're at a low velocity at main deploy (ms)
BOOSTER_MAIN_TO_LANDED_LOCKOUT = 5000

# Stores a small jerk value (m/s^3)
BOOSTER_FIRST_SEPARATION_JERK_THRESHOLD = 300

# Stores a small jerk value (m/s^3)
BOOSTER_DROGUE_JERK_THRESHOLD = 200

# Stores a small jerk value (m/s^3)
BOOSTER_MAIN_JERK_THRESHOLD = 300
