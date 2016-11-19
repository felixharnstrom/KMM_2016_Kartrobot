from pid import Pid
import time

sensor1 = 30    # Front
sensor2 = 30    # Back

controller = Pid()
controller.setpoint = 1
controller.set_tunings(10,0,0)
controller.set_sample_time(500)
controller.set_output_limits(0,2)

while 1:
    # Assuming following wall to the left:
    # If sensor1 > sensor2 (i.e. the ratio is < 1), we are pointing away from the wall. We shold therefore steer to the left.
    # ratio > 1 -> Steer right
    # ratio < 1 -> Steer left
    # ratio = 1 -> Straight ahead
    controller.set_mode(0)
    target = float(input("Target: "))
    controller.set_mode(1)
    while (sensor1 != target):
        print ("S: ", sensor1)
        print ("T: ", target)
        if(target > sensor1):
            sensor1 += 0.5
        elif (target < sensor1):
            sensor1 -= 0.5
        ratio = sensor2 / sensor1
        controller.input_data = ratio
        controller.compute()
        print (controller.output_data)
        time.sleep(0.5)