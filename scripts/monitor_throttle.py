#!/usr/bin/python

# This script is meant to be used to tally good throttle values
# returned from the Udacity simlator.  Make certain that you have
# chmod +x monitor_throttle.py and when you have the simulator
# running do the following:
#
#  rostopic echo /vehicle/throttle_report | ./monitor_throttle.py

import fileinput
import sys

total_num_valid  = 0.0
period_num_valid = 0.0
total_count = 0
period = 100
eps = 1e-5

if __name__ == "__main__":
    for line in fileinput.input():
        total_count += 1
        toks = line.split(" ")
        if toks[0] == "data:":
            if float(toks[1]) > eps:
                total_num_valid += 1.0
                period_num_valid += 1.0

        if total_count % period == 0:
            print "total %f   period %f" % (total_num_valid / total_count, \
                                            period_num_valid / period)
            period_num_valid   = 0
            
