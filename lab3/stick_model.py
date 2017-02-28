#!/usr/bin/python

import tkinter as tk
from tkinter import *
from math import cos, sin, pi
from time import sleep, time
from serial import Serial
import sys

class Robot:

    def click(self, event):

        if time() - self.last_t > 0.5:
            x = int((event.x - self.x0) / self.mm_to_px) + 500
            y = int((self.y0 - event.y) / self.mm_to_px) + 500
            d = x.to_bytes(2, 'big') + y.to_bytes(2, 'big')
            print(x, y, d)
            self.ser.write(d)
            self.last_t = time()


    def __init__(self):
        self.last_t = time()

        if len(sys.argv) != 3:
            print("You need port and baudrate")
            return

        mm_to_px = 1.2
        self.mm_to_px = mm_to_px
        self._mutex = False
        w = 700
        x0 = int(w/2)
        y0 = w
        self.x0 = x0
        self.y0 = y0
        l0 = 143
        l1 = 152
        l2 = 112
        master = Tk()
        canvas = Canvas(master, width=w, height=w)
        canvas.pack()
        self.canvas = canvas
        #canvas.bind('<Button-1>', self.click)
        canvas.bind('<Motion>', self.click)

        self.ser = Serial(sys.argv[1], sys.argv[2], timeout=0.01)

        master.update()

        # Xmm, Ymm, Theta1, Theta2
        while True:
            try:
                d = self.ser.readline()
                if d:
                    line = d.decode('utf-8')
                    values = [float(v.strip('\r\n')) for v in line.split(",")]
                    x3 = values[0]
                    y3 = values[1]
                    t1 = values[2]

                    canvas.delete(ALL)
                    x1 = x0
                    y1 = y0 - (l0 * mm_to_px)
                    canvas.create_line(x0, y0, x1, y1, fill='red', width=20*mm_to_px)
                    x2 = x1 + -sin(pi * t1 / 180) * l1 * mm_to_px
                    y2 = y1 + -cos(pi * t1 / 180) * l1 * mm_to_px
                    canvas.create_line(x1, y1, x2, y2, fill='red', width=20*mm_to_px)
                    canvas.create_line(x2, y2, x0 + x3 * mm_to_px, y0 - y3 * mm_to_px, fill='white', width=20*mm_to_px)
                    canvas.create_line(x0 - 20*mm_to_px, y0, x1 + 120*mm_to_px, y0, fill='black', width=20*mm_to_px)
                    canvas.create_oval(x1 - 10*mm_to_px, y1 - 10*mm_to_px, x1 + 10*mm_to_px, y1 + 10*mm_to_px, fill='black')
                    canvas.create_oval(x2 - 10*mm_to_px, y2 - 10*mm_to_px, x2 + 10*mm_to_px, y2 + 10*mm_to_px, fill='black')

                master.update()
            except IndexError:
                print(d)
                continue
            except ValueError:
                print(d)
                continue

if __name__ == "__main__":
    r = Robot()
