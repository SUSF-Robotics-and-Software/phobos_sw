'''
Generates an extensive script which drives the rover at 0.1 m/s across a range
of ackerman crabs.
'''

import numpy as np

def main():
    '''
    Generate the script
    '''

    time_s = 1.0
    script = '# Generated from scripts/gen/gen_loco_ctrl_text_07.py\n'

    # Generate curvs from -6.5 to 6.5 (wheelbase to wheelbase)
    crabs_rad = np.linspace(-np.pi, np.pi, 20)

    for crab in crabs_rad:
        script += get_ack(time_s, speed_ms=0.1, crab_rad=crab)
        time_s += 2.0

    with open('scripts/loco_ctrl_test_07.prs', 'w') as f:
        f.write(script)

def get_ack(time_s, speed_ms=0.0, curv_m=0.0, crab_rad=0.0):
    return '''
TIME: {
    "LocoCtrlMnvr": {
        "Ackerman": {
            "speed_ms": SPEED,
            "curv_m": CURV,
            "crab_rad": CRAB
        }
    }
};

    '''.replace('TIME', str(time_s))\
        .replace('SPEED', str(speed_ms))\
        .replace('CURV', str(curv_m))\
        .replace('CRAB', str(crab_rad))

if __name__ == '__main__':
    main()
