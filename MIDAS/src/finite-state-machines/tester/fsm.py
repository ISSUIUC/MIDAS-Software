from enum import Enum
from dataclasses import dataclass
import fsm_thresholds as thresholds
from typing import TypedDict

class FSMState(Enum):
    STATE_IDLE = 0
    STATE_FIRST_BOOST = 1
    STATE_BURNOUT = 2
    STATE_COAST = 3
    STATE_SUSTAINER_IGNITION = 4
    STATE_SECOND_BOOST = 5
    STATE_FIRST_SEPARATION = 6
    STATE_APOGEE = 7
    STATE_DROGUE_DEPLOY = 8
    STATE_DROGUE = 9
    STATE_MAIN_DEPLOY = 10
    STATE_MAIN = 11
    STATE_LANDED = 12 
    FSM_STATE_COUNT = 13

FSM_STATE_TO_STRING = {
    FSMState.STATE_IDLE: "IDLE",
    FSMState.STATE_FIRST_BOOST: "FIRST_BOOST",
    FSMState.STATE_BURNOUT: "BURNOUT",
    FSMState.STATE_COAST: "COAST",
    FSMState.STATE_APOGEE: "APOGEE",
    FSMState.STATE_DROGUE_DEPLOY: "DROGUE_DEPLOY",
    FSMState.STATE_DROGUE: "DROGUE",
    FSMState.STATE_MAIN_DEPLOY: "MAIN_DEPLOY",
    FSMState.STATE_MAIN: "MAIN",
    FSMState.STATE_LANDED: "LANDED",
    FSMState.STATE_SUSTAINER_IGNITION: "SUSTAINER_IGNITION",
    FSMState.STATE_SECOND_BOOST: "SECOND_BOOST",
    FSMState.STATE_FIRST_SEPARATION: "FIRST_SEPARATION",
    FSMState.FSM_STATE_COUNT: "STATE_COAST"
}

class StateEstimate(TypedDict):
    acceleration: float
    vertical_speed: float
    jerk: float
    current_time: float
    altitude: float
    state: str

@dataclass
class SustainerFSM:
    state: FSMState = FSMState.STATE_IDLE

    launch_time: float = 0.0
    burnout_time: float = 0.0
    sustainer_ignition_time: float = 0.0
    second_boost_time: float = 0.0
    coast_time: float = 0.0
    drogue_time: float = 0.0
    apogee_time: float = 0.0
    main_time: float = 0.0
    main_deployed_time: float = 0.0
    landed_time: float = 0.0
    first_separation_time: float = 0.0

    def tick_fsm(self, state_estimate: StateEstimate) -> None:
        acceleration: float = state_estimate['acceleration']
        vertical_speed: float = state_estimate['vertical_speed']
        jerk: float = state_estimate['jerk']
        current_time_s: float = state_estimate['current_time']
        altitude: float = state_estimate['altitude']
        
        current_time = 1000 * current_time_s

        reason_transition = ""

        match self.state:
            case FSMState.STATE_IDLE:
                if acceleration > thresholds.SUSTAINER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD:
                    self.launch_time = current_time
                    self.state = FSMState.STATE_FIRST_BOOST
                    reason_transition = f"Transitioned IDLE to FIRST_BOOST due to high acceleration Acceleration is currently {acceleration}m/s^2)"

            case FSMState.STATE_FIRST_BOOST:
                # if acceleration spike was too brief then go back to idle
                if ((acceleration < thresholds.SUSTAINER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD) and ((current_time - self.launch_time) < thresholds.SUSTAINER_IDLE_TO_FIRST_BOOST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_IDLE
                    reason_transition = f"Transitioned FIRST_BOOST to IDLE due to short spike of acceleration. Acceleration is currently {acceleration}m/s^2 and it has been {current_time - self.launch_time}ms since launch_time"
                
                # once acceleartion decreases to a the threshold go on the next state
                elif (acceleration < thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD):
                    self.burnout_time = current_time
                    self.state = FSMState.STATE_BURNOUT
                    reason_transition = f"Transitioned FIRST_BOOST TO BURNOUT due to low acceleration. Acceleration is currently {acceleration}m/s^2"
            case FSMState.STATE_BURNOUT:
                # if low acceleration is too brief than go on to the previous state
                if ((acceleration >= thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD) and ((current_time - self.burnout_time) < thresholds.SUSTAINER_COAST_TIME)):
                    self.state = FSMState.STATE_FIRST_BOOST
                    reason_transition = f"Transitioned BURNOUT TO FIRST_BOOST due to short dip of acceleration. Acceleration is currently {acceleration}m/s^2 and it has been {current_time - self.burnout_time}ms since burnout_time"
                
                # if in burnout for long enough then go on to the next state (time transition)
                elif ((current_time - self.burnout_time) > thresholds.SUSTAINER_COAST_TIME):
                    self.sustainer_ignition_time = current_time
                    self.state = FSMState.STATE_SUSTAINER_IGNITION
                    reason_transition = f"Transitioned BURNOUT TO SUSTAINER_IGNITION due to long enough time after burnout. It has been {current_time - self.burnout_time}ms since burnout_time"
                
            case FSMState.STATE_SUSTAINER_IGNITION:
                # another time transition into coast after a certain amount of time
                if ((current_time - self.sustainer_ignition_time) > thresholds.SUSTAINER_IGNITION_TO_COAST_TIMER_THRESHOLD):
                    self.coast_time = current_time
                    self.state = FSMState.STATE_COAST
                    reason_transition = f"Transitioned SUSTAINER_IGNITION TO COAST due to long enough time after sustainer igition (fail to ignite sustainer). It has been f{current_time - self.sustainer_ignition_time}ms since sustainer_ignition_time"
                
                # once a high enough acceleration is detected then go to next state
                elif (acceleration > thresholds.SUSTAINER_IGNITION_TO_SECOND_BOOST_ACCELERATION_THRESHOLD):
                    self.second_boost_time = current_time
                    self.state = FSMState.STATE_SECOND_BOOST
                    reason_transition = f"Transitioned SUSTAINER_IGNITION TO SECOND_BOOST due to high acceleration. Acceleration is currently {acceleration}m/s^2"

            case FSMState.STATE_SECOND_BOOST:
                # if high accleration is too brief then return to previous state
                if ((acceleration < thresholds.SUSTAINER_IGNITION_TO_SECOND_BOOST_ACCELERATION_THRESHOLD) and ((current_time - self.second_boost_time) < thresholds.SUSTAINER_IGNITION_TO_SECOND_BOOST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_SUSTAINER_IGNITION
                    reason_transition = f"Transitioned SECOND_BOOST TO SUSTAINER_IGNITION due short spike of acceleration. Acceleration is currently {acceleration}m/s^2 and it has been {current_time - self.second_boost_time}ms since second_boost_time"

                # if low acceleration detected go to next state
                elif (acceleration < thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD):
                    self.coast_time = current_time
                    self.state = FSMState.STATE_COAST
                    reason_transition = f"Transitioned SECOND_BOOST TO COAST due to low acceleration. Acceleration is currently {acceleration}m/s^2"
            case FSMState.STATE_COAST:
                # if the low acceleration detected was too brief then return to previous state
                if ((acceleration > thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD) and ((current_time - self.coast_time) < thresholds.SUSTAINER_SECOND_BOOST_TO_COAST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_SECOND_BOOST
                    reason_transition = f"Transitioned COAST TO SECOND_BOOST due to short dip of acceleration. Acceleration is currently {acceleration}m/s^2 and it has been {current_time - self.coast_time}ms since coast_time"

                # if speed slows down enough then go on to the next stage
                elif (vertical_speed <= thresholds.SUSTAINER_COAST_TO_APOGEE_VERTICAL_SPEED_THRESHOLD):
                    self.apogee_time = current_time
                    self.state = FSMState.STATE_APOGEE
                    reason_transition = f"Transitioned COAST TO APOGEE due to low vertical speed. Vertical speed is currently {vertical_speed}m/s"
            case FSMState.STATE_APOGEE:
                # if the slow speed was too brief then return to previous state
                if ((vertical_speed) > thresholds.SUSTAINER_APOGEE_BACKTO_COAST_VERTICAL_SPEED_THRESHOLD and ((current_time - self.apogee_time) < thresholds.SUSTAINER_APOGEE_CHECK_THRESHOLD)):
                    self.state = FSMState.STATE_COAST
                    reason_transition = f"Transitioned APOGEE TO COAST due to short length of vertical speed. Vertical speed is currently {vertical_speed}m/s and it has been {(current_time - self.apogee_time)}ms since apogee"

                # transition to next state after a certain amount of time
                elif ((current_time - self.apogee_time) > thresholds.SUSTAINER_APOGEE_TIMER_THRESHOLD):
                    self.drogue_time = current_time
                    self.state = FSMState.STATE_DROGUE_DEPLOY
                    reason_transition = f"Transitioned APOGEE TO DROGUE_DEPLOY due to length of time. It has been {current_time - self.apogee_time}ms since apogee_time"
            case FSMState.STATE_DROGUE_DEPLOY:
                if ((current_time - self.drogue_time) < thresholds.SUSTAINER_PYRO_FIRING_TIME_MINIMUM):
                    pass

                # if detected a sharp change in jerk then go to next state
                elif (abs(jerk) > thresholds.SUSTAINER_DROGUE_JERK_THRESHOLD):
                    self.state = FSMState.STATE_DROGUE
                    reason_transition = f"Transitioned DROGUE_DEPLOY TO DROGUE due to large magnitude of jerk. Experienced a jerk of {jerk}m/s^3"

                # if no transtion after a certain amount of time then just move on to next state
                elif ((current_time - self.drogue_time) > thresholds.SUSTAINER_DROGUE_TIMER_THRESHOLD):
                    self.state = FSMState.STATE_DROGUE
                    reason_transition = f"Transitioned DROGUE_DEPLOY TO DROGUE due to enough time passed. It has been {current_time - self.drogue_time}ms since drogue_time"
            case FSMState.STATE_DROGUE:
                # if altitude low enough then next state
                if (altitude <= thresholds.SUSTAINER_MAIN_DEPLOY_ALTITUDE_THRESHOLD) and (current_time - self.drogue_time) > thresholds.SUSTAINER_MAIN_DEPLOY_DELAY_AFTER_DROGUE:
                    self.state = FSMState.STATE_MAIN_DEPLOY
                    self.main_time = current_time
                    reason_transition = f"Transitioned DROGUE TO MAIN_DEPLOY due to low enough altitude. Altitude is currently {altitude}m"
            case FSMState.STATE_MAIN_DEPLOY:
                # if detected a sharp change in jerk then go to the next state
                if ((current_time - self.drogue_time) < thresholds.SUSTAINER_PYRO_FIRING_TIME_MINIMUM):
                    pass

                elif (abs(jerk) > thresholds.SUSTAINER_MAIN_JERK_THRESHOLD):
                    self.state = FSMState.STATE_MAIN
                    self.main_deployed_time = current_time
                    reason_transition = "Transitioned MAIN_DEPLOY TO MAIN due to large magnitude of jerk. Experienced a jerk of {jerk}m/s^3"

                # if no transtion after a certain amount of time then just move on to next state
                elif ((current_time - self.main_time) > thresholds.SUSTAINER_MAIN_TO_MAIN_DEPLOY_TIMER_THRESHOLD):
                    self.state = FSMState.STATE_MAIN
                    reason_transition = f"Transitioned MAIN_DEPLOY TO MAIN due to long enough time. It has been {current_time - self.main_time}ms since main_time"
            case FSMState.STATE_MAIN:
                # if slowed down enough then go on to the next state
                if (abs(vertical_speed) <= thresholds.SUSTAINER_LANDED_VERTICAL_SPEED_THRESHOLD) and (current_time - self.main_time) > thresholds.SUSTAINER_MAIN_TO_LANDED_LOCKOUT:
                    self.landed_time = current_time
                    self.state = FSMState.STATE_LANDED
                    reason_transition = f"Transitioned MAIN TO LANDED due to low enough vertical speed. Vertical speed is currently {vertical_speed}m/s"
            case FSMState.STATE_LANDED:
                # if the slow speed was too brief then return to previous state
                if ((abs(vertical_speed) > thresholds.SUSTAINER_LANDED_TO_MAIN_VERTICAL_SPEED_THRESHOLD) and ((current_time - self.landed_time) > thresholds.SUSTAINER_LANDED_TIMER_THRESHOLD)):
                    self.state = FSMState.STATE_MAIN
                    reason_transition = f"Transitioned LANDED TO MAIN due to short length of low vertical speed. Vertical speed is currently {vertical_speed}m/s and it has been {current_time - self.landed_time}ms since landed_time"
            case _:
                pass

        transition_ret = reason_transition

        if transition_ret != "":
            transition_ret = reason_transition + f" at time = {current_time}ms"

        return self.state, transition_ret, current_time
    
@dataclass
class BoosterFsm:
    state: FSMState = FSMState.STATE_IDLE

    launch_time: float = 0.0
    burnout_time: float = 0.0
    sustainer_ignition_time: float = 0.0
    second_boost_time: float = 0.0
    coast_time: float = 0.0
    drogue_time: float = 0.0
    apogee_time: float = 0.0
    main_time: float = 0.0
    main_deployed_time: float = 0.0
    landed_time: float = 0.0
    first_separation_time: float = 0.0
    stage_sep_time: float = 0.0

    def tick_fsm(self, state_estimate: StateEstimate) -> None:
        acceleration: float = state_estimate['acceleration']
        vertical_speed: float = state_estimate['vertical_speed']
        jerk: float = state_estimate['jerk']
        current_time_s: float = state_estimate['current_time']
        altitude: float = state_estimate['altitude']
        
        current_time = 1000 * current_time_s

        reason_transition = ""

        match self.state:
            case FSMState.STATE_IDLE:
                if acceleration > thresholds.BOOSTER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD:
                    self.launch_time = current_time
                    self.state = FSMState.STATE_FIRST_BOOST
                    reason_transition = f"Transitioned IDLE to FIRST_BOOST due to high acceleration Acceleration is currently {acceleration}m/s^2)"

            case FSMState.STATE_FIRST_BOOST:
                # if acceleration spike was too brief then go back to idle
                if ((acceleration < thresholds.BOOSTER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD) and ((current_time - self.launch_time) < thresholds.BOOSTER_IDLE_TO_FIRST_BOOST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_IDLE
                    reason_transition = f"Transitioned FIRST_BOOST to IDLE due to short spike of acceleration. Acceleration is currently {acceleration}m/s^2 and it has been {current_time - self.launch_time}ms since launch_time"
                
                # once acceleartion decreases to a the threshold go on the next state
                elif (acceleration < thresholds.BOOSTER_COAST_DETECTION_ACCELERATION_THRESHOLD):
                    self.burnout_time = current_time
                    self.state = FSMState.STATE_BURNOUT
                    reason_transition = f"Transitioned FIRST_BOOST TO BURNOUT due to low acceleration. Acceleration is currently {acceleration}m/s^2"
            case FSMState.STATE_BURNOUT:
                # if low acceleration is too brief than go on to the previous state
                if ((acceleration >= thresholds.BOOSTER_COAST_DETECTION_ACCELERATION_THRESHOLD) and ((current_time - self.burnout_time) < thresholds.BOOSTER_FIRST_BOOST_TO_BURNOUT_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_FIRST_BOOST
                    reason_transition = f"Transitioned BURNOUT TO FIRST_BOOST due to short dip of acceleration. Acceleration is currently {acceleration}m/s^2 and it has been {current_time - self.burnout_time}ms since burnout_time"
                
                # if in burnout for long enough then go on to the next state (time transition)
                elif ((current_time - self.burnout_time) > thresholds.BOOSTER_FIRST_BOOST_TO_BURNOUT_TIME_THRESHOLD):
                    self.sustainer_ignition_time = current_time
                    self.stage_sep_time = current_time
                    self.state = FSMState.STATE_FIRST_SEPARATION
                    reason_transition = f"Transitioned BURNOUT TO FIRST_SEPARATION due to long enough time after burnout. It has been {current_time - self.burnout_time}ms since burnout_time"
            case FSMState.STATE_FIRST_SEPARATION:
                if ((current_time - self.stage_sep_time) >= thresholds.BOOSTER_PYRO_FIRING_TIME_MINIMUM):
                    # if jerk is low, go to next state
                    if (abs(jerk) < thresholds.BOOSTER_FIRST_SEPARATION_JERK_THRESHOLD):
                        self.state = FSMState.STATE_COAST
                        reason_transition = f"Transitioned FIRST_SEPARATION to COAST due to low magnitude of jerk. Experienced a jerk of {jerk}m/s^3"

                    # if first separation time threshold passed, go to next state
                    elif ((current_time - self.first_separation_time) > thresholds.BOOSTER_FIRST_SEPARATION_TIME_THRESHOLD):
                        self.state = FSMState.STATE_COAST
                        reason_transition = f"Transitioned FIRST_SEPARATION TO COAST due to long enough time after first separation. It has been {current_time - self.first_separation_time}ms since first_separation_time"

            case FSMState.STATE_COAST:
                if (vertical_speed <= thresholds.BOOSTER_COAST_TO_APOGEE_VERTICAL_SPEED_THRESHOLD):
                    self.apogee_time = current_time
                    self.state = FSMState.STATE_APOGEE
                    reason_transition = f"Transitioned COAST to APOGEE due to a low vertical speed of {vertical_speed}m/s"
            case FSMState.STATE_APOGEE:
                # if the slow speed was too brief then return to previous state
                if ((vertical_speed) > thresholds.BOOSTER_COAST_TO_APOGEE_VERTICAL_SPEED_THRESHOLD and ((current_time - self.apogee_time) < thresholds.BOOSTER_APOGEE_CHECK_THRESHOLD)):
                    self.state = FSMState.STATE_COAST
                    reason_transition = f"Transitioned APOGEE TO COAST due to short length of vertical speed. Vertical speed is currently {vertical_speed}m/s and it has been {(current_time - self.apogee_time)}ms since apogee"

                # transition to next state after a certain amount of time
                elif ((current_time - self.apogee_time) > thresholds.BOOSTER_APOGEE_TIMER_THRESHOLD):
                    self.drogue_time = current_time
                    self.state = FSMState.STATE_DROGUE_DEPLOY
                    reason_transition = f"Transitioned APOGEE TO DROGUE_DEPLOY due to length of time. It has been {current_time - self.apogee_time}ms since apogee_time"
            case FSMState.STATE_DROGUE_DEPLOY:
                # lockout for minimum transition time in DROGUE_DEPLOY state
                if ((current_time - self.drogue_time) >= thresholds.BOOSTER_PYRO_FIRING_TIME_MINIMUM):
                    # if detected a sharp change in jerk then go to next state
                    if (abs(jerk) > thresholds.BOOSTER_DROGUE_JERK_THRESHOLD):
                        self.state = FSMState.STATE_DROGUE
                        reason_transition = f"Transitioned DROGUE_DEPLOY TO DROGUE due to large magnitude of jerk. Experienced a jerk of {jerk}m/s^3"

                    # if no transtion after a certain amount of time then just move on to next state
                    if ((current_time - self.drogue_time) > thresholds.BOOSTER_DROGUE_TIMER_THRESHOLD):
                        self.state = FSMState.STATE_DROGUE
                        reason_transition = f"Transitioned DROGUE_DEPLOY TO DROGUE due to enough time passed. It has been {current_time - self.drogue_time}ms since drogue_time"
            case FSMState.STATE_DROGUE:
                # if altitude low enough and 1 second after drogue deploy then go to next state
                if (altitude <= thresholds.BOOSTER_MAIN_DEPLOY_ALTITUDE_THRESHOLD and (current_time - self.drogue_time) > thresholds.BOOSTER_MAIN_DEPLOY_DELAY_AFTER_DROGUE):
                    self.state = FSMState.STATE_MAIN_DEPLOY
                    self.main_time = current_time
                    reason_transition = f"Transitioned DROGUE TO MAIN_DEPLOY due to low enough altitude. Altitude is currently {altitude}m"
            case FSMState.STATE_MAIN_DEPLOY:
                # Lockout for minimum transition time in MAIN_DEPLOY state
                if ((current_time - self.main_time) >= thresholds.BOOSTER_PYRO_FIRING_TIME_MINIMUM):
                    # if detected a sharp change in jerk then go to the next state
                    if (abs(jerk) > thresholds.BOOSTER_MAIN_JERK_THRESHOLD):
                        self.state = FSMState.STATE_MAIN
                        self.main_deployed_time = current_time
                        reason_transition = "Transitioned MAIN_DEPLOY TO MAIN due to large magnitude of jerk. Experienced a jerk of {jerk}m/s^3"

                    # if no transtion after a certain amount of time then just move on to next state
                    if ((current_time - self.main_time) > thresholds.BOOSTER_MAIN_TO_MAIN_DEPLOY_TIMER_THRESHOLD):
                        self.state = FSMState.STATE_MAIN
                        reason_transition = f"Transitioned MAIN_DEPLOY TO MAIN due to long enough time. It has been {current_time - self.main_time}ms since main_time"
            case FSMState.STATE_MAIN:
                # if slowed down enough then go on to the next state
                if (abs(vertical_speed) <= thresholds.BOOSTER_LANDED_VERTICAL_SPEED_THRESHOLD and (current_time - self.main_deployed_time) > thresholds.BOOSTER_MAIN_TO_LANDED_LOCKOUT):
                    self.landed_time = current_time
                    self.state = FSMState.STATE_LANDED
                    reason_transition = f"Transitioned MAIN TO LANDED due to low enough vertical speed. Vertical speed is currently {vertical_speed}m/s"
            case FSMState.STATE_LANDED:
                # if the slow speed was too brief then return to previous state
                if ((abs(vertical_speed) > thresholds.BOOSTER_LANDED_TO_MAIN_VERTICAL_SPEED_THRESHOLD) and ((current_time - self.landed_time) > thresholds.BOOSTER_LANDED_TIMER_THRESHOLD)):
                    self.state = FSMState.STATE_MAIN
                    reason_transition = f"Transitioned LANDED TO MAIN due to short length of low vertical speed. Vertical speed is currently {vertical_speed}m/s and it has been {current_time - self.landed_time}ms since landed_time"
            case _:
                pass

        transition_ret = reason_transition

        if transition_ret != "":
            transition_ret = reason_transition + f" at time = {current_time}ms"

        return self.state, transition_ret, current_time
    3