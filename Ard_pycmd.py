## program for controlling stepper motor along with gripper

import PyCmdMessenger
import time
arduino = PyCmdMessenger.ArduinoBoard("COM6",baud_rate=9600)


# List of commands and their associated argument formats. These must be in the
# same order as in the sketch.
commands = [["motor1","i"],
            ["motor2","i"],
            ["motor3","i"],
            ["motor1_value_is","i"],
            ["motor2_value_is","i"],
            ["motor3_value_is","i"],
            ["motor1_direction_is","i"],
            ["motor2_direction_is","i"],
            ["motor3_direction_is","i"],
            ["gripper","i"],
            ["pump_is","i"]
           ]

# Initialize the messenger
c = PyCmdMessenger.CmdMessenger(arduino,commands)
def step_up(stx,sty,stz,state):
    c.send("motor1",stx)
    msg1 = c.receive()
    c.send("motor2",sty)
    msg2 = c.receive()
    c.send("motor3",stz)
    msg3 = c.receive()
    c.send("gripper",state)
    msg4 = c.receive()
    return msg1,msg2,msg3,msg4