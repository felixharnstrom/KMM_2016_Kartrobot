from robot_communication import *
from command import Command
from datetime import datetime
from follow_wall import *
import map
import gpio_buttons as gpio
import mode
import robot_map_data

def victory_dance():
    while True:
        handle_command(Command.servo(0))
        time.sleep(0.75)
        handle_command(Command.servo(180))
        time.sleep(0.75)

def autonomous_step(robot : Robot):
    robot.update_pid()
    # Drive forward without crashing in wall
    if robot.goal == Goal.NONE:
        victory_dance()
        return
    
    if robot.goal == Goal.FIND_ISLAND:
        status = robot.follow_wall(99999999, side = "right")
    elif robot.goal == Goal.MAP_ISLAND:
        robot_position = robot.get_position()
        if (robot.start_cell_at_island == approximate_to_cell(robot_position) and robot.has_been_to_other_cell):
            robot.leave_island()
            robot.logger.info("RETURNING HOME!")
            status = robot.follow_wall(999999, side = "left")
        elif (robot.start_cell_at_island != approximate_to_cell(robot_position)):
            robot.has_been_to_other_cell = True
            status = robot.follow_wall(999999, side = "right")
        else:
            status = robot.follow_wall(999999, side = "right")
    elif robot.goal == Goal.RETURN_HOME:
        status = robot.follow_wall(9999999, side = "left")
    else:
        status = robot.follow_wall(9999999, side = "right")
        handle_command(Command.stop_motors())    
        robot.logger.debug(robot.get_position())
        robot.logger.debug(robot.get_driven_dist())
        robot.logger.info("Explore island: " + str(robot.goal == Goal.MAP_ISLAND))
        
    robot.update_map()
        
    if (status == DriveStatus.OBSTACLE_DETECTED):
        robot.logger.info("---------- OBSTACLE DETECTED! \n---------- TURNING LEFT 90 degrees")
        turn_instr = Command.stop_motors()
        handle_command(turn_instr)
        while robot._is_moving(): pass
        ir_right_front = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_right_front_ir())
        ir_left_front = robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_left_front_ir())

        if ir_right_front > robot.TURN_OVERRIDE_DIST and ir_left_front > robot.TURN_OVERRIDE_DIST:
            #robot.stand_perpendicular('left')
            robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
            while robot._is_moving(): pass
            #robot.stand_perpendicular('left')
        elif ir_right_front < robot.TURN_OVERRIDE_DIST and ir_left_front < robot.TURN_OVERRIDE_DIST:
            #robot.stand_perpendicular('right')
            robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
            while robot._is_moving(): pass
            #robot.stand_perpendicular('right')
        else:
            if (ir_right_front > ir_left_front):
                #robot.stand_perpendicular('left')
                robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                while robot._is_moving(): pass
                #robot.stand_perpendicular('left')
            else:
                #robot.stand_perpendicular('right')
                robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
                while robot._is_moving(): pass
                #robot.stand_perpendicular('right')
            robot.update_pid()
            #Test the map functionality
    elif (status == DriveStatus.CORRIDOR_DETECTED_RIGHT):
        robot.logger.info("---------- DETECTED CORRIDOR TO RIGHT! \n---------- TURNING RIGHT 90 degrees")
        turn_instr = Command.stop_motors()
        handle_command(turn_instr)
        while robot._is_moving(): pass
        #robot.stand_perpendicular('left')
        robot.drive_distance(robot.CORRIDOR_TURN_ENTRY_DIST, robot.BASE_SPEED, save_new_distance = True)
        robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
        while robot._is_moving(threshold = 30): pass

        # TODO: Detection works, but seems to commonly result in the robot standing staring at a wall, and obstacle detection.
        """if robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_front_ir()) < 200:
            logger.info("Not a corridor, moving back")
            robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
            robot.stand_perpendicular('right')
            while robot._is_moving(threshold = 30): pass
        else:"""
        # TODO: Find a way to do stand_perpendicular when there is no wall to the left. Or lower CORRIDOR_TURN_EXIT_DIST again.
        # As it is now, a right turn into a single square corridor does not work well.
        robot.drive_distance(robot.CORRIDOR_TURN_EXIT_DIST, robot.BASE_SPEED, save_new_distance = True)
        while robot._is_moving(): pass
        #robot.stand_perpendicular('right')
        #robot.stand_perpendicular('left')
        robot.update_pid()
    elif (status == DriveStatus.CORRIDOR_DETECTED_LEFT):
        robot.logger.info("---------- DETECTED CORRIDOR TO LEFT! \n---------- TURNING LEFT 90 degrees")
        turn_instr = Command.stop_motors()
        handle_command(turn_instr)
        while robot._is_moving(): pass
        robot.drive_distance(robot.CORRIDOR_TURN_ENTRY_DIST, robot.BASE_SPEED, save_new_distance = True)
        robot.turn(Direction.LEFT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
        while robot._is_moving(threshold = 30): pass

        # TODO: Detection works, but seems to commonly result in the robot standing staring at a wall, and obstacle detection.
        if robot._median_sensor(robot.IR_MEDIAN_ITERATIONS, Command.read_front_ir()) < 150:
            robot.logger.info("Not a corridor, moving back")
            robot.turn(Direction.RIGHT, 85, speed = robot.ACCELERATED_SPEED, save_new_angle = True)
            robot.stand_perpendicular('left')
            while robot._is_moving(threshold = 30): pass
        else:
            # TODO: Find a way to do stand_perpendicular when there is no wall to the left. Or lower CORRIDOR_TURN_EXIT_DIST again.
            # As it is now, a right turn into a single square corridor does not work well.
            robot.drive_distance(robot.CORRIDOR_TURN_EXIT_DIST, robot.BASE_SPEED, save_new_distance = True)
            while robot._is_moving(): pass
            #robot.stand_perpendicular('left')
            #robot.stand_perpendicular(right')
        robot.update_pid()
    robot.grid_map.debug_print()

def update_map_status(robot : Robot):
    global grid_map_output
    global robot_distance
    r_pos = robot.get_position()
    robot_xy = map.approximate_to_cell(r_pos)
    robot_map_data.set_grid_map(robot.grid_map.gui_drawable(robot_xy.x, robot_xy.y))
    if robot.get_driven_dist():
        robot_map_data.robot_distance = robot.get_driven_dist()[-1][1]

def main():
    """Main loop"""
    # Init
    init_UARTs()
    gpio.launch_poll_threads()
        
    current_time = datetime.now()     
    last_time = current_time     
    diff_time_trigger = 0.3 #Trigger every 0.3s

    # create logger
    logger = logging.getLogger()
    ch = logging.StreamHandler()

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbosity", type = int, default = 1, choices = range(0, 3), help = "Verbosity level. 0 is lowest and 2 highest. Default 1.")
    args = parser.parse_args()

    verbosity = args.verbosity
    if (verbosity == 0):
        logger.disabled = True
        ch.disabled = True
    elif (verbosity == 1):
        logger.setLevel(logging.INFO)
        ch.setLevel(logging.INFO)
    elif (verbosity == 2):
        logger.setLevel(logging.DEBUG)
        ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)

    # Initiate the robot 
    robot = Robot(logger)
    init_wifi_thread(robot.get_position(), robot.grid_map)

    # Turn LIDAR to 90 degrees
    servo_instr = Command.servo(180)
    handle_command(servo_instr)

    # Wait for LIDAR to be in position
    time.sleep(1)

    mode.set_mode(mode.ControlModeEnums.AUTONOMOUS)
    
    # Loop
    while True:
        # Process messages
        current_time = datetime.now()         
        process_actions()
        
        diff = (current_time - last_time).total_seconds()         
        if (diff >= diff_time_trigger): 
            last_time = current_time
            handle_command(Command.controller_information())
            handle_command(Command.read_gyro())
            handle_command(Command.read_lidar())
            handle_command(Command.read_front_ir())
            handle_command(Command.read_left_front_ir())
            handle_command(Command.read_left_back_ir())
            handle_command(Command.read_right_front_ir())
            handle_command(Command.read_right_back_ir())
            handle_command(Command.read_reflex_left())
            handle_command(Command.read_reflex_right())
            print("Time took:", (datetime.now()-last_time).total_seconds())
           # Update motor and sensor values
            
        # Autonomous step
        if mode.get_mode() == mode.ControlModeEnums.AUTONOMOUS:
            if robot == None:
                robot = Robot(logger)
            update_map_status(robot)
            autonomous_step(robot)
        else:
            robot = None
            
    close_UARTs()
    

if __name__ == "__main__":
    main()
