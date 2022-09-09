
from XPlaneUdp import XPlaneUdp
from simple_pid import PID
import time

# Example how to use:
# You need a running xplane in your network. 
if __name__ == '__main__':

  xp = XPlaneUdp()

  try:
    beacon = xp.FindIp()
    print(beacon)
    print()
    
    xp.AddDataRef("sim/flightmodel/position/indicated_airspeed", freq=1000)
    xp.AddDataRef("sim/flightmodel/position/latitude", freq=1000)
    xp.AddDataRef("sim/flightmodel/position/longitude", freq=1000)
    xp.AddDataRef("sim/flightmodel/misc/h_ind",freq=1000)

    xp.AddDataRef("sim/flightmodel/position/true_theta", freq=1000)
    xp.AddDataRef("sim/flightmodel/position/true_phi", freq=1000)
    xp.AddDataRef("sim/flightmodel/position/true_psi", freq=1000)

    heading_control_dataref = "sim/joystick/FC_hdng"
    pitch_control_dataref = "sim/joystick/FC_ptch"
    roll_control_dataref ="sim/joystick/FC_roll"

    gear_dataref = "sim/cockpit2/controls/gear_handle_down"

    throttle_dataref = "sim/cockpit2/engine/actuators/throttle_ratio_all"
    parking_brake_dataref = "sim/flightmodel/controls/parkbrake"

    speed_brake_dataref= "sim/cockpit2/controls/speedbrake_ratio"

    # xp.AddDataRef(heading_dataref)
    # xp.AddDataRef(pitch_dataref)
    # xp.AddDataRef(throttle_dataref)
    # xp.AddDataRef(parking_brake_dataref)
    # xp.AddDataRef(gear_dataref)
    # xp.AddDataRef(roll_dataref)


    values_init = xp.GetValues()

    init_alt = values_init["sim/flightmodel/misc/h_ind"]
    print("INTIAL ALT", init_alt)

    values = values_init
    print(values)

    pitch = values["sim/flightmodel/position/true_theta"]
    roll = values["sim/flightmodel/position/true_phi"]
    yaw = values["sim/flightmodel/position/true_psi"]

    pitch_dataref_value = "sim/flightmodel/position/true_theta"
    yaw_dataref_value = "sim/flightmodel/position/true_psi"
    roll_dataref_value = "sim/flightmodel/position/true_phi"


    target_ele = 10000
    target_alt = init_alt + target_ele
    target_speed = 1000

    alt_controller = PID(5, 0, 250, setpoint=target_alt)
    spd_controller = PID(5, 1, 1, setpoint=target_speed)

    yaw_controller  = PID(1,  0,  10000, setpoint=values_init["sim/flightmodel/position/true_psi"])
    roll_controller = PID(1,  0,  10000, setpoint=values_init["sim/flightmodel/position/true_phi"])

    pitch_trim = 20
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

        if values["sim/flightmodel/position/indicated_airspeed"] > 100:
            pitch_command = alt_controller(values["sim/flightmodel/misc/h_ind"])/100000
            # print("PITCH", pitch_command)

        # if values[pitch_dataref_value] > pitch_trim or values[pitch_dataref_value]< -pitch_trim:
        #     print("TRIM?", values[pitch_dataref_value])
        #     pitch_command = 0


        # if pitch_command > 0.1:
        #     pitch_command = 0.1
        # elif pitch_command <-0.1:
        #     pitch_command = -0.1
        # else:
        #     pitch_command = pitch_command

        throttle = spd_controller(values["sim/flightmodel/position/indicated_airspeed"])/10000

        yaw_command = yaw_controller(values[yaw_dataref_value])/1000000
        roll_command = roll_controller(values[roll_dataref_value])/10000000

        if throttle > throttle_trim:
            throttle = throttle_trim
            speed_brake = 0

        if values["sim/flightmodel/position/indicated_airspeed"] > target_speed:
            throttle = 0
            speed_brake = spd_controller(values["sim/flightmodel/position/indicated_airspeed"])
            # print("BRAKE", speed_brake)

        xp.WriteDataRef(heading_control_dataref, yaw_command)
        xp.WriteDataRef(pitch_control_dataref, pitch_command)
        xp.WriteDataRef(roll_control_dataref, roll_command)

        # print("SET GEAR")
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

        # print(values)
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