## program for controlling the vehicle

import PyCmdMessenger
import time
arduino = PyCmdMessenger.ArduinoBoard("COM7",baud_rate=9600)


# List of commands and their associated argument formats. These must be in the
# same order as in the sketch.
commands = [["Dc1","i"],
            ["Dc2","i"],
            ["Dc1_value_is","i"],
            ["Dc2_value_is","i"],
            ["Dc1_direction_is","i"],
            ["Dc2_direction_is","i"],
           ]

# Initialize the messenger
c = PyCmdMessenger.CmdMessenger(arduino,commands)
def step_up(dc1,dc2):
    c.send("Dc1",dc1)
    msg1 = c.receive()
    c.send("Dc2",dc2)
    msg2 = c.receive()
    return msg1,msg2