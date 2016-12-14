import threading
import tkinter
import os


class gui_thread(threading.Thread):
    """Initiates the GUI."""

    def __init__(self, queue_input, queue_output):
        """Creates GUI thread with the queues used to pass data to/from the GUI.

        Keyword arguments:
        queue_input -- this queue will be used for all communication TO the GUI
        queue_output -- this queue will be used for all communication FROM the GUI
        """

        # Make pythons thread library happy
        threading.Thread.__init__(self)

        # Make variables accessible to the whole class
        self.queue = queue_input
        self.gui = None
        self.pressed_keys = {"left":False, "right":False, "up":False, "down":False}
 
    def key_pressed(self, event):
        """Callback for key press."""
        button = event.keysym
        if button == "Left" and not self.pressed_keys["left"]:
            self.pressed_keys["left"] = True
            self.send_command("key_left_p")
        elif button == "Right" and not self.pressed_keys["right"]:
            self.pressed_keys["right"] = True
            self.send_command("key_right_p")
        elif button == "Up" and not self.pressed_keys["up"]:
            self.pressed_keys["up"] = True
            self.send_command("key_up_p")
        elif button == "Down" and not self.pressed_keys["down"]:
            self.pressed_keys["down"] = True
            self.send_command("key_down_p")


    def key_released(self, event):
        """Callback for key release."""
        button = event.keysym
        if button == "Left":
            self.pressed_keys["left"] = False
            self.send_command("key_left_r")
        elif button == "Right":
            self.pressed_keys["right"] = False
            self.send_command("key_right_r")
        elif button == "Up":
            self.pressed_keys["up"] = False
            self.send_command("key_up_r")
        elif button == "Down":
            self.pressed_keys["down"] = False
            self.send_command("key_down_r")

    def send_command(self, mode):
        self.queue.put(mode)

    def receive_command(self, command):
        if type(command)==list:
            if command[0] == "draw":
                # It was a draw command.
                # Instruct the gui to place a marker on its canvas at the point specified in "command"
                self.place_marker_on_canvas(command[1], command[2])
            elif command[0] == "set_motors":
                self.left_motor_speed.set(command[1])
                self.right_motor_speed.set(command[2])
            elif command[0] == "set_servo":
                self.servo_angle.set(command[1])
            elif command[0] == "set_sensors":
                sensor_data_dict = command[1]
                self.ir_front_left.set(sensor_data_dict["IR_LEFT_FRONT"])
                self.ir_front_right.set(sensor_data_dict["IR_RIGHT_FRONT"])
                self.ir_back_left.set(sensor_data_dict["IR_LEFT_BACK"])
                self.ir_back_right.set(sensor_data_dict["IR_RIGHT_BACK"])
                self.ir_behind.set(sensor_data_dict["IR_BACK"])
                self.lidar.set(sensor_data_dict["LIDAR"])
                self.gyro.set(sensor_data_dict["GYRO"])
                #self.reflex_right.set(sensor_data_dict["REFLEX_RIGHT"])
                #self.reflex_left.set(sensor_data_dict["REFLEX_LEFT"])
                #TODO: Send this from PC in the sensor_data dict
                #self.distance.set(sensor_data_dict["DISTANCE"])
        return

    def run(self):
        """Initiates the GUI with previously specified queues."""

        # The usual gui setup.
        self.gui = tkinter.Tk()
        self.gui.minsize(800, 400)
        self.gui.grid()

        # Bind keys
        self.gui.bind("<Left>", self.key_pressed)
        self.gui.bind("<Right>", self.key_pressed)
        self.gui.bind("<Up>", self.key_pressed)
        self.gui.bind("<Down>", self.key_pressed)
        self.gui.bind("<KeyRelease-Left>", self.key_released)
        self.gui.bind("<KeyRelease-Right>", self.key_released)
        self.gui.bind("<KeyRelease-Up>", self.key_released)
        self.gui.bind("<KeyRelease-Down>", self.key_released)

        # We need to know how big our window got.
        self.gui.update()
        window_width = self.gui.winfo_width()
        window_height = self.gui.winfo_height()

        # Split the window into frames
        canvas_frame = tkinter.Frame(self.gui, width=window_width/2, height=window_height)
        canvas_frame.grid(row=0, column=0, rowspan=2)

        sensor_frame = tkinter.Frame(self.gui, width=window_width/4, height=window_height)
        sensor_frame.grid(row=0, column=2, rowspan=2,sticky = tkinter.N)
        self.gui.grid_columnconfigure(2, weight=1)

        button_frame = tkinter.Frame(self.gui, width=window_width/4, height=window_height/2)
        button_frame.grid(row=0, column=1)

        debug_frame = tkinter.Frame(self.gui, width=window_width/4, height=window_height/2)
        debug_frame.grid(row=1, column=1)

        # Place a canvas into canvas_frame
        self.canvas = tkinter.Canvas(canvas_frame, width=window_width/2, height=window_height)
        self.canvas.place(relx=0.5, rely=1, anchor=tkinter.CENTER)

        # Create a bunch of buttons in the left frame.
        # Each button will place a command in the queue using send_command

        # Mode switching buttons
        mode_var = tkinter.IntVar()
        R1 = tkinter.Radiobutton(button_frame, text="Manual", variable=mode_var, value=1,
                         command=lambda message="mode_manual": self.send_command(message))
        R1.pack()

        R2 = tkinter.Radiobutton(button_frame, text="Autonomous", variable=mode_var, value=2,
                         command=lambda message="mode_autonomous": self.send_command(message))
        R2.pack()

        mode_var.set(1)

        
        self.left_motor_speed = tkinter.DoubleVar()
        self.motor_left_scale = tkinter.Scale(debug_frame, variable=self.left_motor_speed, orient=tkinter.HORIZONTAL,
                                              from_=-100, to=100, state=tkinter.DISABLED)
        self.motor_left_scale.pack()
        self.insert_text(" Left Motor Speed ", debug_frame)

        self.right_motor_speed = tkinter.DoubleVar()
        self.motor_right_scale = tkinter.Scale(debug_frame, variable=self.right_motor_speed, orient=tkinter.HORIZONTAL,
                                               from_=-100, to=100, state=tkinter.DISABLED)
        self.motor_right_scale.pack()
        self.insert_text(" Right Motor Speed ", debug_frame)
        
        self.servo_angle = tkinter.DoubleVar()
        self.servo_angle_scale = tkinter.Scale(debug_frame, variable=self.servo_angle, orient=tkinter.HORIZONTAL,
                                                 to=180, state=tkinter.DISABLED)
        self.servo_angle_scale.pack()
        self.insert_text(" Servo Angle ", debug_frame)

        # Quit button
        quit_button = tkinter.Button(button_frame, text='QUIT',
                                     command=lambda message="quit": self.send_command(message))
        quit_button.pack()

        #Insert manual instructions
        self.insert_text("Use the arrow keys to\nmove in manual mode.", button_frame)

        #Setup sensor frame
        sensor_frame.grid_columnconfigure(1, weight=1)

        reflex_text = tkinter.Label(sensor_frame,text="[Reflex Sensors]")
        reflex_text.grid(row=0, column=0, columnspan=2, pady = 10)
        self.reflex_left = tkinter.DoubleVar()
        reflex_left_lab = tkinter.Label(sensor_frame, textvariable=self.reflex_left)
        reflex_left_lab.grid(row=1, column=0, padx = 18)
        self.reflex_right = tkinter.DoubleVar()
        reflex_right_lab = tkinter.Label(sensor_frame,textvariable=self.reflex_right)
        reflex_right_lab.grid(row=1, column=1, padx= 18)
        
        ir_front_text = tkinter.Label(sensor_frame,text="[IR sensors front]")
        ir_front_text.grid(row=2, column=0, columnspan=2, pady = 10)
        self.ir_front_left = tkinter.DoubleVar()
        ir_front_left_lab = tkinter.Label(sensor_frame, textvariable=self.ir_front_left)
        ir_front_left_lab.grid(row=3, column=0, padx = 18)
        self.ir_front_right = tkinter.DoubleVar()
        ir_front_right_lab = tkinter.Label(sensor_frame,textvariable=self.ir_front_right)
        ir_front_right_lab.grid(row=3, column=1, padx= 18)

        ir_back_text = tkinter.Label(sensor_frame,text="[IR sensors back]")
        ir_back_text.grid(row=4, column=0, columnspan=2, pady = 10)
        self.ir_back_left = tkinter.DoubleVar()
        ir_back_left_lab = tkinter.Label(sensor_frame, textvariable=self.ir_back_left)
        ir_back_left_lab.grid(row=5, column=0, padx = 18)
        self.ir_back_right = tkinter.DoubleVar()
        ir_back_right_lab = tkinter.Label(sensor_frame,textvariable=self.ir_back_right)
        ir_back_right_lab.grid(row=5, column=1, padx= 18)

        ir_behind_text = tkinter.Label(sensor_frame,text="[IR backside]")
        ir_behind_text.grid(row=6, column=0, columnspan=2, pady = 10)
        self.ir_behind = tkinter.DoubleVar()
        ir_behind_lab = tkinter.Label(sensor_frame, textvariable=self.ir_behind)
        ir_behind_lab.grid(row=7, column=0, columnspan=2, padx = 18)

        lidar_text = tkinter.Label(sensor_frame,text="[LIDAR]")
        lidar_text.grid(row=8, column=0, columnspan=2, pady = 10)
        self.lidar = tkinter.DoubleVar()
        lidar_lab = tkinter.Label(sensor_frame, textvariable=self.lidar)
        lidar_lab.grid(row=9, column=0, columnspan=2, padx = 18)

        gyro_text = tkinter.Label(sensor_frame,text="[GYRO]")
        gyro_text.grid(row=10, column=0, columnspan=2, pady = 10)
        self.gyro = tkinter.DoubleVar()
        gyro_lab = tkinter.Label(sensor_frame, textvariable=self.gyro)
        gyro_lab.grid(row=11, column=0, columnspan=2, padx = 18)
        
        distance_text = tkinter.Label(sensor_frame,text="[Distance Traveled]")
        distance_text.grid(row=12, column=0, columnspan=2, pady = 10)
        self.distance = tkinter.DoubleVar()
        distance_lab = tkinter.Label(sensor_frame, textvariable=self.distance)
        distance_lab.grid(row=13, column=0, columnspan=2, padx = 18)

        #Setup exit routine
        self.gui.protocol("WM_DELETE_WINDOW", lambda message="quit": self.send_command(message))

        # Start tkinters mainloop
        self.gui.mainloop()

    def insert_text(self, txt, target):
        """Inserts and packs text on a target canvas. Supports multiline with \n in txt."""
        lines = txt.split("\n")
        longest_line_len = max(len(line) for line in lines)
        text = tkinter.Text(target, height=len(lines), width=longest_line_len)
        for line in lines:
            text.insert(tkinter.INSERT, line)
        text.config(state=tkinter.DISABLED, pady=5)
        text.pack()

    def place_marker_on_canvas(self, x, y):
        self.canvas.create_rectangle(x - 5, y - 5,
                                     x + 5, y + 5,
                                     fill="black")

        pass

    def draw_map(self, map : list):
        for x,mapx in enumerate(map):
            for y,mapy in enumerate(mapx):
                if mapy==1:
                    self.place_marker_on_canvas(100+x*10,100+y*10)

