'''
Generates an extensive script which drives the rover at 0.1 m/s across a range
of ackerman curvatures.
'''

import numpy as np

def main():
    '''
    Generate the script
    '''

    time_s = 1.0
    script = '# Generated from scripts/gen/gen_loco_ctrl_text_05.py\n'

    # Generate curvs from -6.5 to 6.5 (wheelbase to wheelbase)
    curvs_m = np.linspace(-6.5, 6.5, 20)

    for curv in curvs_m:
        script += get_ack(time_s, curv, 0.1)
        time_s += 2.0

    with open('scripts/loco_ctrl_test_05.prs', 'w') as f:
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
