# One sample of the drone's gas sensors: SGX MiCS-6814 (CO, NO2, NH3) and Winsen MQ-136 (H2S).

std_msgs/Header header        # stamp = when the air was sampled
string[] species              # CO | NO2 | NH3 | H2S
float32[] ppm                 # 0 below the sensor's floor; the top of its range when saturated
bool[] saturated              # at the top of the sensor's range: the true value may be higher
