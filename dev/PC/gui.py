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
        print("Command", mode, "put into queue.")
        self.queue.put(mode)

    def receive_command(self, command):
        if type(command)==list and command[0] == "draw":
            # It was a draw command.
            # Instruct the gui to place a marker on its canvas at the point specified in "command"
            self.place_marker_on_canvas(command[1], command[2])
        elif type(command) == list and command[0] == "set_motors":
            self.left_motor_speed.set(command[1])
            self.right_motor_speed.set(command[2])
        elif type(command) == list and command[0] == "set_servo":
            self.servo_angle.set(command[1])

    def run(self):
        """Initiates the GUI with previously specified queues."""

        # The usual gui setup.
        self.gui = tkinter.Tk()
        self.gui.minsize(900, 500)
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
        canvas_frame = tkinter.Frame(self.gui, width=4*window_width/8, height=window_height/2)
        canvas_frame.grid(row=0, column=0)

        button_frame = tkinter.Frame(self.gui, width=window_width/8, height=window_height/2)
        button_frame.grid(row=0, column=1)

        debug_frame = tkinter.Frame(self.gui, width=window_width/8, height=window_height/2)
        debug_frame.grid(row=1, column=1)

        #dummy_frame = tkinter.Frame(self.gui, width=(window_width / 4), height=window_height)
        #dummy_frame.grid(row=1, column=1)

        # Place a canvas into canvas_frame
        self.canvas = tkinter.Canvas(canvas_frame, width=4*window_width/8, height=window_height)
        self.canvas.place(relx=0.5, rely=1, anchor=tkinter.CENTER)

        # Create a bunch of buttons in the left frame.
        # Each button will place a command in the queue using send_command

        # Mode switching buttons
        mode_var = tkinter.IntVar()
        R1 = tkinter.Radiobutton(button_frame, text="Manual", variable=mode_var, value=1,
                         command=lambda message="mode_manual": self.send_command(message))
        R1.pack()

        R2 = tkinter.Radiobutton(button_frame, text="Automnomous", variable=mode_var, value=2,
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
                                               from_=-90, to=90, state=tkinter.DISABLED)
        self.servo_angle_scale.pack()
        self.insert_text(" Servo Angle ", debug_frame)

        # Buttons for manual control
        #forward_command_button = tkinter.Button(button_frame, text='Forward',
        #                                     command=lambda message="forward": self.send_command(message))
        #left_command_button = tkinter.Button(button_frame, text='Left',
        #                                     command=lambda message="left": self.send_command(message))
        #right_command_button = tkinter.Button(button_frame, text='Right',
        #                                      command=lambda message="right": self.send_command(message))
        #stop_command_button = tkinter.Button(button_frame, text='Stop',
        #                                     command=lambda message="stop_motors": self.send_command(message))

        # Quit button
        quit_button = tkinter.Button(button_frame, text='QUIT',
                                     command=lambda message="quit": self.send_command(message))

        # Place them into the UI
        #forward_command_button.pack()
        #left_command_button.pack()
        #right_command_button.pack()
        #stop_command_button.pack()
        quit_button.pack()

        self.insert_text("Use the arrow keys to\nmove in manual mode.", button_frame)

        self.gui.protocol("WM_DELETE_WINDOW", lambda message="quit": self.send_command(message))

        # Start tkinters mainloop
        self.gui.mainloop()

    def insert_text(self, txt, target):
        """Inserts and packs text on a target canvas. Supports multiline with \n in txt."""
        lines = txt.split("\n")
        longest_line_len = max(len(line) for line in lines)
        self.left_motor_text = tkinter.Text(target, height=len(lines), width=longest_line_len)
        for line in lines:
            self.left_motor_text.insert(tkinter.INSERT, line)
        self.left_motor_text.config(state=tkinter.DISABLED)
        self.left_motor_text.pack()

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

