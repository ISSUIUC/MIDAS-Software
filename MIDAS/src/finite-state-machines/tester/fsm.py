from enum import Enum
from dataclasses import dataclass
import fsm_thresholds as thresholds
from typing import TypedDict

class FSMState(Enum):
    STATE_IDLE = 0
    STATE_FIRST_BOOST = 1
    STATE_BURNOUT = 2
    STATE_COAST = 3
    STATE_APOGEE = 4
    STATE_DROGUE_DEPLOY = 5
    STATE_DROGUE = 6
    STATE_MAIN_DEPLOY = 7
    STATE_MAIN = 8
    STATE_LANDED = 9 
    STATE_SUSTAINER_IGNITION = 10
    STATE_SECOND_BOOST = 11
    STATE_FIRST_SEPARATION = 12
    FSM_STATE_COUNT = 13

FSM_STATE_TO_STRING = {
    FSMState.STATE_IDLE: "IDLE",
    FSMState.STATE_FIRST_BOOST: "FIRST_BOOST",
    FSMState.STATE_BURNOUT: "BURNOUT",
    FSMState.STATE_COAST: "COAST",
    FSMState.STATE_APOGEE: "APOGEE",
    FSMState.STATE_DROGUE_DEPLOY: "DROGIE_DEPLOY",
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
                    reason_transition = "Transitioned IDLE to FIRST_BOOST due to high acceleration"

            case FSMState.STATE_FIRST_BOOST:
                # if acceleration spike was too brief then go back to idle
                if ((acceleration < thresholds.SUSTAINER_IDLE_TO_FIRST_BOOST_ACCELERATION_THRESHOLD) and ((current_time - self.launch_time) < thresholds.SUSTAINER_IDLE_TO_FIRST_BOOST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_IDLE
                    reason_transition = f"Transitioned FIRST_BOOST to IDLE due to short spike of acceleration"
                
                # once acceleartion decreases to a the threshold go on the next state
                elif (acceleration < thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD):
                    self.burnout_time = current_time
                    self.state = FSMState.STATE_BURNOUT
                    reason_transition = "Transitioned FIRST_BOOST TO BURNOUT due to low acceleration"
            case FSMState.STATE_BURNOUT:
                # if low acceleration is too brief than go on to the previous state
                if ((acceleration >= thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD) and ((current_time - self.burnout_time) < thresholds.SUSTAINER_FIRST_BOOST_TO_BURNOUT_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_FIRST_BOOST
                    reason_transition = "Transitioned BURNOUT TO FIRST_BOOST due to short dip of acceleration"
                
                # if in burnout for long enough then go on to the next state (time transition)
                elif ((current_time - self.burnout_time) > thresholds.SUSTAINER_FIRST_BOOST_TO_BURNOUT_TIME_THRESHOLD):
                    self.sustainer_ignition_time = current_time
                    self.state = FSMState.STATE_SUSTAINER_IGNITION
                    reason_transition = "Transitioned BURNOUT TO SUSTAINER_IGNITION due to long enough time after burnout"
                
            case FSMState.STATE_SUSTAINER_IGNITION:
                # another time transition into coast after a certain amount of time
                if ((current_time - self.sustainer_ignition_time) > thresholds.SUSTAINER_IGNITION_TO_COAST_TIMER_THRESHOLD):
                    self.coast_time = current_time
                    self.state = FSMState.STATE_COAST
                    reason_transition = "Transitioned SUSTAINER_IGNITION TO COAST due to long enough time after sustainer igition (fail to ignite sustainer)"
                
                # once a high enough acceleration is detected then go to next state
                elif (acceleration > thresholds.SUSTAINER_IGNITION_TO_SECOND_BOOST_ACCELERATION_THRESHOLD):
                    self.second_boost_time = current_time
                    self.state = FSMState.STATE_SECOND_BOOST
                    reason_transition = "Transitioned SUSTAINER_IGNITION TO SECOND_BOOST due to high acceleration"

            case FSMState.STATE_SECOND_BOOST:
                # if high accleration is too brief then return to previous state
                if ((acceleration < thresholds.SUSTAINER_IGNITION_TO_SECOND_BOOST_ACCELERATION_THRESHOLD) and ((current_time - self.second_boost_time) < thresholds.SUSTAINER_IGNITION_TO_SECOND_BOOST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_SUSTAINER_IGNITION
                    reason_transition = "Transitioned SECOND_BOOST TO SUSTAINER_IGNITION due short spike of acceleration"

                # if low acceleration detected go to next state
                elif (acceleration < thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD):
                    self.coast_time = current_time
                    self.state = FSMState.STATE_COAST
                    reason_transition = "Transitioned SECOND_BOOST TO COAST due to low acceleration"
            case FSMState.STATE_COAST:
                # if the low acceleration detected was too brief then return to previous state
                if ((acceleration > thresholds.SUSTAINER_COAST_DETECTION_ACCELERATION_THRESHOLD) and ((current_time - self.coast_time) < thresholds.SUSTAINER_SECOND_BOOST_TO_COAST_TIME_THRESHOLD)):
                    self.state = FSMState.STATE_SECOND_BOOST
                    reason_transition = "Transitioned COAST TO SECOND_BOOST due to short dip of acceleration"

                # if speed slows down enough then go on to the next stage
                elif (vertical_speed <= thresholds.SUSTAINER_COAST_TO_APOGEE_VERTICAL_SPEED_THRESHOLD):
                    self.apogee_time = current_time
                    self.state = FSMState.STATE_APOGEE
                    reason_transition = "Transitioned COAST TO APOGEE due to low vertical speed"
            case FSMState.STATE_APOGEE:
                # if the slow speed was too brief then return to previous state
                if ((vertical_speed) > thresholds.SUSTAINER_APOGEE_BACKTO_COAST_VERTICAL_SPEED_THRESHOLD and ((current_time - self.apogee_time) < thresholds.SUSTAINER_APOGEE_CHECK_THRESHOLD)):
                    self.state = FSMState.STATE_COAST
                    reason_transition = f"Transitioned APOGEE TO COAST due to short length of vertical speed, {vertical_speed} m/s. "

                # transition to next state after a certain amount of time
                elif ((current_time - self.apogee_time) > thresholds.SUSTAINER_APOGEE_TIMER_THRESHOLD):
                    self.drogue_time = current_time
                    self.state = FSMState.STATE_DROGUE_DEPLOY
                    reason_transition = "Transitioned APOGEE TO DROGUE_DEPLOY due to length of time"
            case FSMState.STATE_DROGUE_DEPLOY:
                # if detected a sharp change in jerk then go to next state
                if (abs(jerk) < thresholds.SUSTAINER_DROGUE_JERK_THRESHOLD):
                    self.state = FSMState.STATE_DROGUE
                    reason_transition = "Transitioned DROGUE_DEPLOY TO DROGUE due to large magnitude of jerk"

                # if no transtion after a certain amount of time then just move on to next state
                if ((current_time - self.drogue_time) > thresholds.SUSTAINER_DROGUE_TIMER_THRESHOLD):
                    self.state = FSMState.STATE_DROGUE
                    reason_transition = "Transitioned DROGUE_DEPLOY TO DROGUE due to enough time passed"
            case FSMState.STATE_DROGUE:
                # if altitude low enough then next state
                if (altitude <= thresholds.SUSTAINER_MAIN_DEPLOY_ALTITUDE_THRESHOLD):
                    self.state = FSMState.STATE_MAIN_DEPLOY
                    self.main_time = current_time
                    reason_transition = "Transitioned DROGUE TO MAIN_DEPLOY due to low enough altitude"
            case FSMState.STATE_MAIN_DEPLOY:
                # if detected a sharp change in jerk then go to the next state
                if (abs(jerk) < thresholds.SUSTAINER_MAIN_JERK_THRESHOLD):
                    self.state = FSMState.STATE_MAIN
                    reason_transition = "Transitioned MAIN_DEPLOY TO MAIN due to large magnitude of jerk"

                # if no transtion after a certain amount of time then just move on to next state
                if ((current_time - self.main_time) > thresholds.SUSTAINER_MAIN_TO_MAIN_DEPLOY_TIMER_THRESHOLD):
                    self.state = FSMState.STATE_MAIN
                    reason_transition = "Transitioned MAIN_DEPLOY TO MAIN due to long enough time"
            case FSMState.STATE_MAIN:
                # if slowed down enough then go on to the next state
                if (abs(vertical_speed) <= thresholds.SUSTAINER_LANDED_VERTICAL_SPEED_THRESHOLD):
                    self.landed_time = current_time
                    self.state = FSMState.STATE_LANDED
                    reason_transition = "Transitioned MAIN TO LANDED due to long enough vertical speed"
            case FSMState.STATE_LANDED:
                # if the slow speed was too brief then return to previous state
                if ((abs(vertical_speed) > thresholds.SUSTAINER_LANDED_VERTICAL_SPEED_THRESHOLD) and ((current_time - self.landed_time) > thresholds.SUSTAINER_LANDED_TIMER_THRESHOLD)):
                    self.state = FSMState.STATE_MAIN
                    reason_transition = "Transitioned LANDED TO MAIN due to short length of low vertical speed"
            case _:
                pass

        transition_ret = reason_transition

        if transition_ret != "":
            transition_ret = reason_transition + f" at time = {current_time}ms"

        return self.state, transition_ret
    