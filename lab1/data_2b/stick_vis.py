#!/usr/bin/python3.5

import tkinter as tk
from tkinter import *
from math import cos, sin, pi
from time import sleep
import pandas
import sys

def main():
    if len(sys.argv) != 2:
        print("You need filename")
        return

    mm_to_px = 1.2
    w = 700
    x0 = int(w/2)
    y0 = w
    l0 = 143
    l1 = 152
    l2 = 112
    master = Tk()
    canvas = Canvas(master, width=w, height=w)
    canvas.pack()

    df = pandas.read_csv(sys.argv[1])

    # Xmm, Ymm, Theta1, Theta2
    for values in df.values:
        try:
            x3 = values[1]
            y3 = values[2]
            t1 = values[3]

            canvas.delete(ALL)
            x1 = x0
            y1 = y0 - (l0 * mm_to_px)
            canvas.create_line(x0, y0, x1, y1, fill='red', width=20*mm_to_px)
            x2 = x1 + -sin(pi * t1 / 180) * l1 * mm_to_px
            y2 = y1 + -cos(pi * t1 / 180) * l1 * mm_to_px
            canvas.create_line(x1, y1, x2, y2, fill='red', width=20*mm_to_px)
            canvas.create_line(x2, y2, x0 + x3 * mm_to_px, y0 - y3 * mm_to_px, fill='red', width=20*mm_to_px)
            canvas.create_line(x0 - 20*mm_to_px, y0, x1 + 120*mm_to_px, y0, fill='black', width=20*mm_to_px)
            canvas.create_oval(x1 - 10*mm_to_px, y1 - 10*mm_to_px, x1 + 10*mm_to_px, y1 + 10*mm_to_px, fill='black')
            canvas.create_oval(x2 - 10*mm_to_px, y2 - 10*mm_to_px, x2 + 10*mm_to_px, y2 + 10*mm_to_px, fill='black')
            sleep(0.01)

            master.update()
        except IndexError:
            continue
        except ValueError:
            continue

if __name__ == "__main__":
    main()
