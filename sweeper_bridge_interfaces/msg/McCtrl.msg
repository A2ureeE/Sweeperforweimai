uint8 brake        # brake command: 0=off, 1=on
uint8 gear         # gear: 0=neutral, 1=reverse, 2=forward
uint16 rpm         # motor RPM (0-6000)
float32 angle      # steering angle (degrees)
uint16 angle_speed # steering angular velocity
bool sweep         # one-key sweep: true=active
int32 sweep_mode   # sweep mode: 0=brush, 1=brush+water
bool enable_main_brush      # main brush motor
bool enable_vacuum          # vacuum motor
bool enable_dust_shake      # dust shake motor
bool enable_main_brush_pole # main brush actuator
bool enable_flap_pole       # front flap actuator
bool enable_side_brush     # side brush motor
bool enable_water_pump      # water pump motor
