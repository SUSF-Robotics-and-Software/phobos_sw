import zmq
import json
import toml
import math
from adafruit_servokit import ServoKit
import sys
import signal
import time

# Global variables, that can be killed from the sighandler
ROVER = None
# Global flag to stop execution
STOP_EXEC = False

# signal handler to destroy sockets on CTRL-C
def exit_sig_handler(signal_number, frame):
    global STOP_EXEC
    STOP_EXEC = True

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
            'StrFL': 0,
            'StrML': 1,
            'StrRL': 2,
            'StrFR': 3,
            'StrMR': 4,
            'StrRR': 5,
            'ArmBase': 0,
            'ArmShoulder': 1,
            'ArmElbow': 2,
            'ArmWrist': 3,
            'ArmGrabber': 4
        }

    def init_eqpt(self):
        self.channels = 16
        # Start a servo kit with 16 channels available for each board
        self.servo_kits = [
            ServoKit(channels=self.channels, address=self.mech_exec['board_addresses'][0]),
            ServoKit(channels=self.channels, address=self.mech_exec['board_addresses'][1])
        ]

    def get_motor(self, name, group):
        '''
        This function returns the servo that needs to be used
        '''
        motor_setting = self.load_setting(name, group)
        # Determine which servo kit needs to be used and in which board this servo is located
        servo_kit = self.servo_kits[motor_setting['Servo_Kit']]
        if group == "Str" or group == "Arm":
            # Set the actuation range of the motor only if it is a steer motor or a Arm motor
            servo_kit.servo[motor_setting['Channel']].actuation_range = motor_setting["Actuation_Range"]
            # Set the pulse width of the motor
            servo_kit.servo[motor_setting['Channel']].set_pulse_width_range(motor_setting["Pulse_Width"][0], motor_setting["Pulse_Width"][1])
        
            return servo_kit.servo[motor_setting['Channel']]
        else:
            return servo_kit.continuous_servo[motor_setting['Channel']]

    def load_setting(self, name, group):
        '''
        Loads the parameters for a given motor
        '''
        # This variable is used to track the index in which a specific motor is inside the given list (of parameters) in the toml file
        motor_idx = self.motor_id_dict[name]

        # Dictionary containing main information about the servo
        motor_setting = {
            "Channel" : self.mech_exec[group.lower() + '_idx_map'][motor_idx][1],
            "Actuation_Range" : self.mech_exec['str_act_range_sk'][motor_idx],
            "Pulse_Width" : [
                int(self.mech_exec['str_pw_range_min'][motor_idx]), 
                int(self.mech_exec['str_pw_range_max'][motor_idx])
            ],
            "Servo_Kit" : self.mech_exec[group.lower() + '_idx_map'][motor_idx][0]
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
                rad_to_sk_map = self.mech_exec[f'{group.lower()}_ang_rad_to_sk_coeffs'][
                    self.motor_id_dict[act_id]
                ]
                pos_sk = rad_to_sk_map[0] * position_rad + rad_to_sk_map[1]
                # Clamp to min/max values
                pos_sk = max(
                    self.mech_exec[f'{group.lower()}_ang_min_sk'][self.motor_id_dict[act_id]],
                    min(
                        self.mech_exec[f'{group.lower()}_ang_max_sk'][self.motor_id_dict[act_id]],
                        pos_sk
                    )
                )
                print(f'{act_id}: raw: {position_rad}, sk: {pos_sk}')
                # Set the position of the servo in degreee, the demands give the position in radians so RAD_TO_DEGREE_CONV is used to convert
                self.get_motor(act_id, group).angle = pos_sk

        # Actuate speed demands
        for act_id, speed_rads in dems['speed_rads'].items():
            # Get motor group
            group = act_id[:3]

            if group == 'Drv':
                rate_to_sk_map = self.mech_exec['drv_rate_norm_to_sk_coeffs'][
                    self.motor_id_dict[act_id]
                ]
                rate_sk = rate_to_sk_map[0] * speed_rads + rate_to_sk_map[1]
                # Clamp to min/max values
                rate_sk = max(
                    self.mech_exec['drv_rate_min_sk'][self.motor_id_dict[act_id]],
                    min(
                        self.mech_exec['drv_rate_max_sk'][self.motor_id_dict[act_id]],
                        rate_sk
                    )
                )
                print(f'{act_id}: {rate_sk}')
                self.get_motor(act_id, group).throttle = rate_sk

    def stop(self):
        '''
        Bring the rover to a complete stop.
        '''

        print('Mechanisms.stop() called')

        for motor in self.motor_id_dict.keys():
            group = motor[:3]

            if group == 'Drv':
                self.get_motor(motor, group).throttle = \
                    self.mech_exec['drv_rate_norm_to_sk_coeffs'][self.motor_id_dict[motor]][1]

def run(mechanisms):
    '''
    Run the rover.
    '''
    global STOP_EXEC

    # Create zmq context
    ZMQ_CONTEXT = zmq.Context()

    # Open mechanisms server
    MECH_REP = ZMQ_CONTEXT.socket(zmq.REP)
    MECH_REP.RCVTIMEO = 1000 # 1 second timeout
    MECH_REP.bind(mechanisms.mech_exec['demands_endpoint'])
    MECH_PUB = ZMQ_CONTEXT.socket(zmq.PUB)
    MECH_PUB.bind(mechanisms.mech_exec['sensor_data_endpoint'])

    print('MechServer started')

    # Run flag
    run_controller = True

    print('Starting main control loop')
    while run_controller:
        # Run mechanisms task
        run_controller &= handle_mech(mechanisms, MECH_REP, MECH_PUB)

        sys.stdout.flush()

        # Check global stop flag
        run_controller &= not STOP_EXEC

    # Stop the rover
    mechanisms.stop()

    # Close sockets
    MECH_REP.close()
    MECH_PUB.close()

    # Destroy context
    ZMQ_CONTEXT.destroy()

def handle_mech(mechanisms, mech_rep, mech_pub):
    '''
    Handle mechanisms commands and publish mech data
    '''
    # Flag indicating whether or not to stop the rover
    stop = False

    # Get mechanisms demands from the rep socket
    try:
        dems_str = mech_rep.recv_string()
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
        mech_rep.send_string('"DemsOk"')

        # Actuate
        mechanisms.actuate_mech_dems(mech_dems)

    return True

def main():
    global ROVER

    # SIGINT handler that will properly close sockets on CTRL-C/Z
    signal.signal(signal.SIGINT, exit_sig_handler)
    signal.signal(signal.SIGTSTP, exit_sig_handler)
    # Create phobos and run the rover exec code
    ROVER = Mechanisms('../params/mech_exec.toml', '../params/loco_ctrl.toml')

    ROVER.stop()

    run(ROVER)

if __name__ == '__main__':

    main()
