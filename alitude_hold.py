
from XPlaneUdp import XPlaneUdp
from simple_pid import PID
import time

# Example how to use:
# You need a running xplane in your network. 

def set_datarefs(xp, ref_list):
  for ref in ref_list:
    xp.AddDataRef(ref, freq=1000)
  return xp


if __name__ == '__main__':

  xp = XPlaneUdp()

  try:
    beacon = xp.FindIp()
    print(beacon)
    print()
    
    ref_list = [
    "sim/flightmodel/position/indicated_airspeed",
    "sim/flightmodel/position/latitude",
    "sim/flightmodel/position/longitude",
    "sim/flightmodel/misc/h_ind",
    "sim/flightmodel/position/true_theta",
    "sim/flightmodel/position/true_phi",
    "sim/flightmodel/position/true_psi", 
    ]
    set_datarefs(xp, ref_list)

    # heading_control_dataref = "sim/joystick/FC_hdng"
    # pitch_control_dataref = "sim/joystick/FC_ptch"
    # roll_control_dataref ="sim/joystick/FC_roll"

    heading_control_dataref = "sim/cockpit2/controls/yoke_heading_ratio"
    pitch_control_dataref = "sim/cockpit2/controls/yoke_pitch_ratio"
    roll_control_dataref ="sim/cockpit2/controls/yoke_roll_ratio"

    # sim/cockpit2/controls/total_heading_ratio	float	y	[-1..1]	Total rudder control input (sum of user pedal plus autopilot servo plus artificial stability)
    # sim/cockpit2/controls/total_pitch_ratio	float	y	[-1..1]	Total pitch control input (sum of user yoke plus autopilot servo plus artificial stability)
    # sim/cockpit2/controls/total_roll_ratio	float	y	[-1..1]	Total roll control input (sum of user yoke plus autopilot servo plus artificial stability)

    gear_dataref = "sim/cockpit2/controls/gear_handle_down"
    throttle_dataref = "sim/cockpit2/engine/actuators/throttle_ratio_all"
    parking_brake_dataref = "sim/flightmodel/controls/parkbrake"
    speed_brake_dataref= "sim/cockpit2/controls/speedbrake_ratio"


    values_init = xp.GetValues()
    init_alt = values_init["sim/flightmodel/misc/h_ind"]
    print("Initial Altitude: ", init_alt)

    values = values_init

    pitch = values["sim/flightmodel/position/true_theta"]
    roll = values["sim/flightmodel/position/true_phi"]
    yaw = values["sim/flightmodel/position/true_psi"]

    pitch_dataref_value = "sim/flightmodel/position/true_theta"
    yaw_dataref_value = "sim/flightmodel/position/true_psi"
    roll_dataref_value = "sim/flightmodel/position/true_phi"


    target_ele = 20000
    target_alt = init_alt + target_ele
    target_speed = 500

    spd_controller = PID(5, 1, 100, setpoint=target_speed)

    # yaw_controller  = PID(1,  0,  1000000, setpoint=values_init["sim/flightmodel/position/true_psi"])
    # roll_controller = PID(1,  0,  1000000, setpoint=values_init["sim/flightmodel/position/true_phi"])
    yaw_controller  = PID(1,  0,  3000000, setpoint=169)
    roll_controller = PID(1,  0,  3000000, setpoint=0)
    alt_controller  = PID(5,  0,  250,     setpoint=target_alt)

    throttle_trim = 1
    pitch_command = 0
    speed_brake = 0

    xp.WriteDataRef(parking_brake_dataref, 0)
    time.sleep(1)
    while True:
      try:
        values = xp.GetValues()

        if values["sim/flightmodel/misc/h_ind"] > init_alt + 500:
            print("RETRACT LG")
            gear = 0
        else:
            gear = 1

        if values["sim/flightmodel/position/indicated_airspeed"] > 150:
            pitch_command = alt_controller(values["sim/flightmodel/misc/h_ind"])/90000

        throttle = spd_controller(values["sim/flightmodel/position/indicated_airspeed"])/1000
        yaw_command = yaw_controller(values[yaw_dataref_value])/5000000
        roll_command = roll_controller(values[roll_dataref_value])/20000000

        if throttle > throttle_trim:
            throttle = throttle_trim
            speed_brake = 0

        if values["sim/flightmodel/position/indicated_airspeed"] > target_speed:
            throttle = 0
            speed_brake = spd_controller(values["sim/flightmodel/position/indicated_airspeed"])/10000
    

        xp.WriteDataRef(heading_control_dataref, yaw_command)
        xp.WriteDataRef(pitch_control_dataref, pitch_command)
        xp.WriteDataRef(roll_control_dataref, roll_command)

        xp.WriteDataRef(gear_dataref, gear)
        xp.WriteDataRef(throttle_dataref, throttle)
        xp.WriteDataRef(speed_brake_dataref, speed_brake)

        print("ANGLE COMMANDS P/Y/R", pitch_command, yaw_command, roll_command)
        print("THROTTLE: ", throttle)   
        print("BRAKE", speed_brake)
        print("TARGET ALT: ", target_alt)
        print("GOBAL POSITION AND HEADING (Lat, Long, Alt, Pitch, Roll, Yaw): (%4f, %4f, %4f, %4f, %4f, %4f) \n"\
                % (values["sim/flightmodel/position/latitude"], values["sim/flightmodel/position/longitude"], values["sim/flightmodel/misc/h_ind"], values["sim/flightmodel/position/true_theta"], values["sim/flightmodel/position/true_phi"], values["sim/flightmodel/position/true_psi"]))
        print("AIR SPEED m/s", values["sim/flightmodel/position/indicated_airspeed"])

        time.sleep(0.001)

      except XPlaneTimeout:
        print("XPlane Timeout")
        exit(0)

  except XPlaneVersionNotSupported:
    print("XPlane Version not supported.")
    exit(0)

  except XPlaneIpNotFound:
    print("XPlane IP not found. Probably there is no XPlane running in your local network.")
    exit(0)

xp = XPlaneUdp()