'''
Generates an extensive script which exercises a wide range of loco_ctrl's 
capabilities.
'''

import numpy as np

def main():
    '''
    Generate the script
    '''

    time_s = 1.0
    script = '# Generated from scripts/gen/gen_loco_ctrl_text_02.py\n'

    # Generate speeds from -0.175 to 0.175 m/s (20) with 3 s spacing
    speeds_ms = np.linspace(-0.175, 0.175, 20)

    for speed in speeds_ms:
        script += get_ack(time_s, 0.0, speed)
        time_s += 3.0

    # Generate curvs from -6.5 to 6.5 (wheelbase to wheelbase)
    curvs_m = np.linspace(-6.5, 6.5, 20)

    for curv in curvs_m:
        script += get_ack(time_s, curv, 0.0)
        time_s += 3.0

    with open('scripts/loco_ctrl_test_02.prs', 'w') as f:
        f.write(script)

def get_ack(time_s, curv_m, speed_ms):
    return '''
TIME: {
    "type": "MNVR",
    "payload": {
        "mnvr_type": "ACKERMAN",
        "curv_m": CURV,
        "speed_ms": SPEED
    }
};

    '''.replace('TIME', str(time_s)).replace('CURV', str(curv_m)).replace('SPEED', str(speed_ms))

if __name__ == '__main__':
    main()
