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

    def key_pressed(self, event):
        """Callback for key press."""
        button = event.keysym
        if button == "Left":
            self.send_command("left")
        elif button == "Right":
            self.send_command("right")
        elif button == "Up":
            self.send_command("forward")
        elif button == "Down":
            self.send_command("back")


    def key_released(self, event):
        """Callback for key release."""
        button = event.keysym
        buttons = {"Left", "Right", "Up", "Down"}
        if button in buttons:
            self.send_command("stop_motors")

    def send_command(self, mode):
        print("Command", mode, "put into queue.")
        self.queue.put(mode)

    def receive_command(self, command):
        if type(command)==list and command[0]=="draw":
            # It was a draw command.
            # Instruct the gui to place a marker on its canvas at the point specified in "command"
            self.place_marker_on_canvas(command[1], command[2])
        elif type(command) == list and command[0]=="set_motors":
            self.left_motor_speed.set(command[1])
            self.right_motor_speed.set(command[2])

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
        button_frame = tkinter.Frame(self.gui, width=(window_width / 4), height=window_height)
        button_frame.grid(row=0, column=0)

        canvas_frame = tkinter.Frame(self.gui, width=2 * window_width / 4, height=window_height)
        canvas_frame.grid(row=0, column=1)

        debug_frame = tkinter.Frame(self.gui, width=window_width / 4, height=window_height)
        debug_frame.grid(row=0, column=2)

        # Place a canvas into canvas_frame
        self.canvas = tkinter.Canvas(canvas_frame, width=2 * window_width / 4, height=window_height)
        self.canvas.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

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
        self.motor_left_scale = tkinter.Scale(debug_frame, variable=self.left_motor_speed, orient=tkinter.HORIZONTAL, state=tkinter.DISABLED)
        self.motor_left_scale.pack()

        self.right_motor_speed = tkinter.DoubleVar()
        self.motor_right_scale = tkinter.Scale(debug_frame, variable=self.right_motor_speed, orient=tkinter.HORIZONTAL,
                                         state=tkinter.DISABLED)
        self.motor_right_scale.pack()

        # Buttons for manual control
        forward_command_button = tkinter.Button(button_frame, text='Forward',
                                             command=lambda message="forward": self.send_command(message))
        left_command_button = tkinter.Button(button_frame, text='Left',
                                             command=lambda message="left": self.send_command(message))
        right_command_button = tkinter.Button(button_frame, text='Right',
                                              command=lambda message="right": self.send_command(message))
        stop_command_button = tkinter.Button(button_frame, text='Stop',
                                             command=lambda message="stop_motors": self.send_command(message))

        # Quit button
        quit_button = tkinter.Button(button_frame, text='QUIT',
                                     command=lambda message="quit": self.send_command(message))

        # Place them into the UI
        forward_command_button.pack()
        left_command_button.pack()
        right_command_button.pack()
        stop_command_button.pack()
        quit_button.pack()

        self.gui.protocol("WM_DELETE_WINDOW", lambda message="quit": self.send_command(message))

        # Start tkinters mainloop
        self.gui.mainloop()

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

