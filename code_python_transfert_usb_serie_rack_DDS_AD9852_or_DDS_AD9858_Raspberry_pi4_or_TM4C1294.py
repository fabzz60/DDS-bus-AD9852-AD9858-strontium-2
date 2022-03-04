# -*- coding: utf-8 -*-
# On importe Tkinter
from tkinter import Tk, StringVar, Label, Entry, Button
from tkinter import*
from functools import partial
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt

    
def packIntegerAsULong(valeur):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.pack('I', valeur)    #should check bounds

def packIntegerAsUInt(valeur):
    """Packs a python 2 byte unsigned integer to an MSP430 unsigned short"""
    return struct.pack('H', valeur)    #should check bounds

def packUnsignedCharAsChar(valeur):
    """Packs a python 1 byte char to an MSP430 unsigned short"""
    return struct.pack('B', valeur)    #should check bounds

def update_label(label, stringvar):
    
    t0 = values.get()
    t17 = var.get()
    ser =serial.Serial(
    port = t0,
    #baudrate = 9600,
    baudrate = t17,
    parity = serial.PARITY_NONE, 
    stopbits = serial.STOPBITS_ONE, 
    bytesize = serial.EIGHTBITS)
    #timeout = 100)
    text = stringvar.get()
    label.config(text=text)
    stringvar.set('data load!')
    t2 = value2.get()
    ser.write(packIntegerAsUInt(t2))
    t = value.get()
    ser.write(packIntegerAsULong(t))
    t3 = value3.get()
    ser.write(packUnsignedCharAsChar(t3))
    print("virtual serial port")
    print(ser.name)
    print("baudrate")
    print(ser.baudrate)
    print("parity")
    print(ser.parity)
    print("stopbits")
    print(ser.stopbits)
    print("bytesize")
    print(ser.bytesize)
    print("timeout")
    print(ser.timeout)

    
root = Tk() # CrÃ©ation de la fenÃªtre racine
root.title("programme pour Rack DDS AD9858 or DDS AD9852 interface Raspberry pi4 ou uC TM4C1294 F.Wiotte DEC 2021")
root.geometry("800x800")
#root.configure(background="none")
frame = Frame(root)
frame.pack()

bottomframe = Frame(root)
bottomframe.pack( side = BOTTOM )

bouton=Button(root, text="quitter", fg="red",font=3,command=root.destroy) # Bouton qui dÃ©truit la fenÃªtre
bouton.pack(side = BOTTOM)       # insÃ¨re le bouton dans la fenÃªtre

values = StringVar(root) 
spin = Spinbox(root,values=('COM1','COM2','COM3','COM4','COM5','COM6','COM7','COM8','COM9','COM10'),fg="green",textvariable=values)
spin.pack(side = BOTTOM)


w = Label(root, text="Baud Rate and Port",fg="green",font=("Helvetica", 10))
w.pack()

var = IntVar(root)
var.set(115200)
spin2 = Spinbox(root, from_=9600, to=115200,increment=100,fg="green",textvariable=var)
spin2.pack(side = BOTTOM)


text = StringVar(root)
label = Label(frame, text='')
entry_name = Entry(frame, textvariable=text)
button = Button(frame, text='charger le programme',fg ='red',font=("Helvetica", 9),
                command=partial(update_label, label, text))

label.grid(column=20, row=80)
entry_name.grid(column=20, row=80)
button.grid(column=10, row=80)

#value = IntVar(root) 
value =IntVar()
value.set(0)
scale1 = Scale(frame, from_=0, to=130000000,resolution=1000,fg="blue", showvalue=True, label='Freq DDS AD9852',
variable=value, tickinterval=4, orient='h')
entry = Entry(frame, textvariable=value)
scale1.grid(row=0, column=0)
entry.grid(row=0, column=1)


#value2 = IntVar(root) 
value2 =IntVar()
value2.set(0)
scale2 = Scale(frame, from_=0, to=4095,resolution=10,fg="blue", showvalue=True, label='Amplitude DDS',
variable=value2, tickinterval=4, orient='h')
entry = Entry(frame, textvariable=value2)
scale2.grid(row=1, column=0)
entry.grid(row=1, column=1)

#value3 = IntVar(root) 
value3 =IntVar()
value3.set(1)
scale3 = Scale(frame, from_=1, to=4,resolution=1,fg="blue", showvalue=True, label='SELECT DDS',
variable=value3, tickinterval=4, orient='h')
entry = Entry(frame, textvariable=value3)
scale3.grid(row=2, column=0)
entry.grid(row=2, column=1)

#plt.show() # affiche la figure a l'ecran
root.mainloop() # Lancement de la boucle principale




