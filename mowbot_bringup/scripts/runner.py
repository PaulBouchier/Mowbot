#!/usr/bin/env python

import sys

argv_pos = 1

class drive_straight:
    def __init__(self, distance):
        self.distance = distance
    def print(self):
        print('drive straight for {} m'.format(self.distance))

class rotate_in_place:
    def __init__(self, angle):
        self.angle = angle
    def print(self):
        print('rotate in place {} deg'.format(self.angle))

def drive():
    m = drive_straight(sys.argv[argv_pos+1])
    moves.append(m)
def rotate():
    m = rotate_in_place(sys.argv[argv_pos+1])
    moves.append(m)
def default():
    print ('Error: unrecognized move: {}'.format(sys.argv[argv_pos]))
    sys.exit()

print ('num args: {}'.format(len(sys.argv)))
print('args: {}'.format(str(sys.argv)))
if len(sys.argv) < 3:
    print('error: must provide pairs of args')
    sys.exit()
if len(sys.argv) % 2 != 1:
    print('error: must provide even number of args')
    sys.exit()
moves = []

switcher = {
    'd': drive,
    'r': rotate
    }

def switch(move_rqst):
    # print(sys.argv[argv_pos])
    return switcher.get(sys.argv[argv_pos], default)()

while (argv_pos < len(sys.argv)):
    switch(sys.argv[argv_pos])
    argv_pos += 2

for m in moves:
    m.print()

