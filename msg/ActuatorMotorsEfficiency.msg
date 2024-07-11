# Motor control message
uint64 timestamp			# time since system start (microseconds)

float32[12] efficiency # range: [-1, 1], where 1 means maximum positive thrust,
                       # -1 maximum negative (if not supported by the output, <0 maps to NaN),
                       # and NaN maps to disarmed (stop the motors)
