#!/usr/bin/env python3

import subprocess
import time
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                    prog='robot spawner',
                    description='Runs ros2 launch spice simulated_robot.launch.py requested number of times')
    parser.add_argument('--number_of_robots')
    args = parser.parse_args()
    if args.number_of_robots is None:
        print('--number_of_robts is an required argument')
        exit(0)
    num_robots = int(args.number_of_robots)
    for i in range(num_robots):
        x = 1.3 + (i % 15)*0.65
        y = 1.3
        cmd = ["ros2", "launch", "spice", "simulated_robot.launch.py", "nr:="+str(i), f'x:={x:.5f}', f"y:={y:.5f}"]
        subprocess.Popen(cmd)
        time.sleep(5)
        
    while True:
        time.sleep(0.1)

