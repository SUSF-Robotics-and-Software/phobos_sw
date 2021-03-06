import zmq
import json
import toml
import math
from adafruit_servokit import ServoKit
import sys

# CONSTANTS
RAD_TO_DEGREE_CONV = 180 / math.pi
MAX_SPEED_RADS = 3.6458 # Maximum drive motor speed in rad/s
# Toggle to disable/enable specific servers

class Mechanisms:
    def __init__(self, mech_exec_path, loco_ctrl_path):
        '''
        Main constructor, used to initialise the rover
        '''
        self.mech_exec = toml.load(mech_exec_path)
        self.loco_ctrl = toml.load(loco_ctrl_path)

        self.init_eqpt()

        # Used to give a numerical value to each motor present on the rover
        self.motor_id_dict = {
            'DrvFL': 0,
            'DrvML': 1,
            'DrvRL': 2,
            'DrvFR': 3,
            'DrvMR': 4,
            'DrvRR': 5,
            'StrFL': 6,
            'StrML': 7,
            'StrRL': 8,
            'StrFR': 9,
            'StrMR': 10,
            'StrRR': 11,
            'ArmBase': 12,
            'ArmShoulder': 13,
            'ArmElbow': 14,
            'ArmWrist': 15,
            'ArmGrabber': 16
        }

    def init_eqpt(self):
        self.channels = 16
        # Start a servo kit with 16 channels available for each board
        self.servo_kits = [ServoKit(channels=self.channels, address=0x40),
                           ServoKit(channels=self.channels, address=0x41)]

    def get_motor(self, name, group):
        '''
        This function returns the servo that needs to be used
        '''
        # Determine which servo kit needs to be used and in which board this servo is located
        servo_kit = self.servo_kits[self.load_setting(name, group)['Servo_Kit']]
        if group == "Str" or group == "Arm":
            # Set the actuation range of the motor only if it is a steer motor or a Arm motor
            servo_kit.servo[self.motor_id_dict[name]].actuation_range = self.load_setting(name, group)["Actuation_Range"]
        # Set the pulse width of the motor
        servo_kit.servo[self.motor_id_dict[name]].set_pulse_width_range(self.load_setting(name, group)["Pulse_Width"][0], self.load_setting(name, group)["Pulse_Width"][1])
        return servo_kit.servo[self.motor_id_dict[name]]

    def load_setting(self, name, group):
        '''
        Loads the parameters for a given motor
        '''
        # This variable is used to track the index in which a specific motor is inside the given list (of parameters) in the toml file
        motor_ID = -1
        if self.motor_id_dict[name] <= 5:
            motor_ID = self.motor_id_dict[name]

        elif self.motor_id_dict[name] >= 6 and self.motor_id_dict[name] <= 11:
            motor_ID = self.motor_id_dict[name] - 6

        elif self.motor_id_dict[name] >= 12:
            motor_ID = self.motor_id_dict[name] - 12

        # Dictionary containing main information about the servo
        motor_setting = {
            "Board_ID" : self.mech_exec['board_addresses'],
            "Channel" : self.mech_exec[group.lower() + '_idx_map'][motor_ID][1],
            "Actuation_Range" : self.mech_exec['str_act_range_sk'][motor_ID],
            "Pulse_Width" : [int(self.mech_exec['str_pw_range_min'][motor_ID]), int(self.mech_exec['str_pw_range_max'][motor_ID])],
            "Servo_Kit" : self.mech_exec[group.lower() + '_idx_map'][motor_ID][0]
        }
        return motor_setting

    def actuate_mech_dems(self, dems):
        '''
        Actuate the given mechanisms demands.
        '''

        # Actuate position demands
        for act_id, position_rad in dems['pos_rad'].items():
            # Get the motor group
            group = act_id[:3]

            if group == 'Str' or group == 'Arm':
                # Set the position of the servo in degreee, the demands give the position in radians so RAD_TO_DEGREE_CONV is used to convert
                self.get_motor(act_id, group).angle = position_rad * RAD_TO_DEGREE_CONV

        # Actuate speed demands
        for act_id, speed_rads in dems['speed_rads'].items():
            # Get motor group
            group = act_id[:3]

            if group == 'Drv':
                self.get_motor(act_id, group).throttle = speed_rads / MAX_SPEED_RADS

    def stop(self):
        '''
        Bring the rover to a complete stop.
        '''

        for motor in self.motor_id_dict.keys():
            group = motor[:3]

            if group == 'Drv':
                self.get_motor(motor, group).throttle = 0

    def pose(self):
        '''
        Get the pose of the rover
        '''
        pass


def run(mechanisms):
    '''
    Run the rover.
    '''

    # Create zmq context
    context = zmq.Context()

    # Open mechanisms server
    mech_rep = context.socket(zmq.REP)
    mech_rep.bind(mechanisms.mech_exec['demands_endpoint'])
    mech_pub = context.socket(zmq.PUB)
    mech_pub.bind(mechanisms.mech_exec['sensor_data_endpoint'])

    print('MechServer started')

    # Run flag
    run_controller = True

    print('Starting main control loop')
    while run_controller:
        # Run mechanisms task
        run_controller &= handle_mech(mechanisms, mech_rep, mech_pub)

        sys.stdout.flush()

    # Close sockets
    mech_rep.close()
    mech_pub.close()

    # Destroy context
    context.destroy()

    sys.exit(0)

def handle_mech(mechanisms, mech_rep, mech_pub):
    '''
    Handle mechanisms commands and publish mech data
    '''
    # Flag indicating whether or not to stop the rover
    stop = False

    # Get mechanisms demands from the rep socket
    try:
        dems_str = mech_rep.recv_string(flags=zmq.NOBLOCK)
        mech_dems = json.loads(dems_str)
    except zmq.Again:
        mech_dems = None
    except zmq.ZMQError as e:
        print(f'MechServer: Error - {e}, ({e.errno})')
        stop = True
        mech_dems = None
    except Exception as e:
        print(f'MechServer Exception: {e}')
        stop = True
        mech_dems = None

    # If no demand
    if mech_dems is None:
        # If an error occured stop the rover
        if stop:
            mechanisms.stop()
    else:
        # Send response to client
        mech_rep.send_string('"Dems Ok"')

        # Actuate
        mechanisms.actuate_mech_dems(mech_dems)

    return True

def main():

    # Create phobos and run the rover exec code
    mechanisms = Mechanisms('../params/mech_exec.toml', '../params/loco_ctrl.toml')

    run(mechanisms)

if __name__ == '__main__':

    main()
