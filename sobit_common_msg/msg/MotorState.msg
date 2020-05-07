time stamp   # motor state is at this time
int32 id            # motor id
int32 goal          # commanded position (in encoder units)
int32 position      # current position (in encoder units)
int32 speed         # current speed (0.111 rpm per unit)
int32 load        # current load - ratio of applied torque over maximum torque
int32 voltage     # current voltage (V)
int32 temperature   # current temperature (degrees Celsius)
bool moving         # whether the motor is currently in motion
