#!/usr/bin/python
# -*- coding: utf-8 -*-
# The wiring for the LCD is as follows:
# 1 : GND
# 2 : 5V
# 3 : Contrast (0-5V)*
# 4 : RS (Register Select)
# 5 : R/W (Read Write)       - GROUND THIS PIN
# 6 : Enable or Strobe
# 7 : Data Bit 0             
# 8 : Data Bit 1             
# 9 : Data Bit 2             
# 10: Data Bit 3             
# 11: Data Bit 4
# 12: Data Bit 5
# 13: Data Bit 6
# 14: Data Bit 7
# 15: LCD Backlight +5V**
# 16: LCD Backlight GND
# programme Python for DDS Rack AD9858-AD9852
import RPi.GPIO as GPIO #Ajout de la bibliothèque GPIO
import time
from subprocess import*
import datetime
import serial
import struct
from pickle import*
from datetime import datetime
#from time import sleep, strtime
import spidev #Ajout de la bibliothèque SPI
GPIO.setmode(GPIO.BOARD) #Définir le mode de numérotation
import spidev #Ajout de la bibliothèque SPI
spi = spidev.SpiDev() # crée l'objet SPI
spi.open(0,1) # demarre spi port 0,(CS) 1
spi.max_speed_hz = 10000000
#cmd ="ip addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1"
GPIO.setwarnings(False)

ser =serial.Serial(
#port ='COM5',
"/dev/ttyS0",
baudrate = 115200, 
parity = serial.PARITY_NONE, 
stopbits = serial.STOPBITS_ONE, 
bytesize = serial.EIGHTBITS)
#timeout = 100
#)
print("virtual serial port")
print(ser.name)
print("baudrate")
print("parity")
print("stopbits")
print(ser.stopbits)
print("bitesize")
print(ser.bytesize)
print("timeout")
print(ser.timeout)

def unpackIntegerAsULong(value):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.unpack('I', value)    #should check bounds
#GPIO.setmode(GPIO.BOARD) #Définir le mode de numérotation
# Define GPIO to LCD mapping

cmd = "ip addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1"
  
def run_cmd(cmd):
    p = Popen(cmd, shell=True, stdout=PIPE)
    output = p.communicate()[0]
    return output

LCD_RS = 37
LCD_E  = 35
LCD_D0 = 40
LCD_D1 = 38
LCD_D2 = 36
LCD_D3 = 32
LCD_D4 = 33
LCD_D5 = 31
LCD_D6 = 29
LCD_D7 = 5
# Define GPIO DDS
reset_DDS = 3
update_DDS = 22
#CS DDS1
CS1 = 11
#CS DDS2
CS2 = 13
#CS DDS3
CS3 = 15
#CS DDS4
CS4 = 16
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
# Define some device constants
LCD_WIDTH = 16  # Maximum characters per line 4x16
LCD_CHR = True
LCD_CMD = False

# adress CFR DDS AD9858
CFR_ADRESS = 0x00
# adress CFR word  =0X58 SYNC clock disable MSB first 2 serial wire
CFR_1 = 0x00
CFR_2 = 0x00
CFR_3 = 0x00
CFR_4 = 0x58

# adress FTWO DDS AD9858
FTW0_ADRESS = 0x03
# adress FTW0 word  10MHz @ 1GHz clock
FTW0_1 = 0x02
FTW0_2 = 0x8F
FTW0_3 = 0x5C
FTW0_4 = 0x28

# adress FTWO DDS AD9852
FTW_ADRESS = 0x02
# adress FTW0 word
FTW0 = 0x07
FTW0 = 0x87
FTW0 = 0x87
FTW0 = 0x87
FTW0 = 0x87
FTW0 = 0x87

# adress PLL DDS AD9852
PLL_ADRESS = 0x07
#PLL value  x17 clk = 340MHz
PLL_0 = 0x10
PLL_1 = 0x51
PLL_2 = 0x01
PLL_3 = 0x60
# adress OSK DDS AD9852
OSK_ADRESS = 0x08
#OSK value
OSK_0 = 0x08
OSK_1 = 0x00
  
GPIO.setup(reset_DDS, GPIO.OUT)  # reset_DDS
GPIO.output(reset_DDS, True)
time.sleep(0.003)
GPIO.output(reset_DDS, False)
time.sleep(1)

GPIO.setup(CS1, GPIO.OUT)  # CS1 DDS 1
GPIO.output(CS1, True)
time.sleep(0.01)
GPIO.output(CS1, False)

GPIO.setup(CS2, GPIO.OUT)  # CS2 DDS 2
GPIO.output(CS2, True)
time.sleep(E_DELAY)
GPIO.output(CS2, False)

GPIO.setup(CS3, GPIO.OUT)  # CS3 DDS 3
GPIO.output(CS3, True)
time.sleep(E_DELAY)
GPIO.output(CS3, False)

GPIO.setup(CS4, GPIO.OUT)  # CS4 DDS 4
GPIO.output(CS4, True)
time.sleep(E_DELAY)
GPIO.output(CS4, False)

# frequence F1 en mémoire
f = open("freq1_memory","rb")
d = load(f)
freq1 = d["freq1"]
f.close()
print("frequence F1 en Hz")
print(freq1)

#frequence F2 en mémoire
f = open("freq2_memory","rb")
d = load(f)
freq2 = d["freq2"]
f.close()
print("frequence F2 en Hz")
print(freq2)

#frequence F3 en mémoire
f = open("freq3_memory","rb")
d = load(f)
freq3 = d["freq3"]
f.close()
print("frequence F3 en Hz")
print(freq3)

#frequence F4 en mémoire
f = open("freq4_memory","rb")
d = load(f)
freq4 = d["freq4"]
f.close()
print("frequence F4 en Hz")
print(freq4)


#amplitude F3 en mémoire
f = open("amp3_memory","rb")
d = load(f)
amp3 = d["amp3"]
f.close()
print("amplitude F3")
print(amp3)

#amplitude F4 en mémoire
f = open("amp4_memory","rb")
d = load(f)
amp4 = d["amp4"]
f.close()
print("amplitude F4")
print(amp4)

#DDS1 AD9858 activation
GPIO.output(CS1, False)  # CS1 DDS 1
GPIO.output(CS2, True) # CS2 DDS 2
GPIO.output(CS3, True) # CS3 DDS 3
FTW = ((freq1 * 4294967296)/1000000000)
#print(int(FTW))
s1 =int(FTW).to_bytes(4,'little')
#print(s1)
list_of_value1 =[CFR_ADRESS, CFR_1, CFR_2, CFR_3, CFR_4]
spi.xfer(list_of_value1)
list_of_value6 = [FTW0_ADRESS,s1[3],s1[2],s1[1],s1[0]]
spi.xfer(list_of_value6)
GPIO.setup(update_DDS, GPIO.OUT)  # update DDS AD9858
GPIO.output(update_DDS, True)
time.sleep(E_DELAY)
GPIO.output(update_DDS, False)
print("data fréquence F1 en mémoire récupéree")

#DDS2 AD9858 activation
GPIO.output(CS1, True)  # CS1 DDS 1
GPIO.output(CS2, False) # CS2 DDS 2
GPIO.output(CS3, True) # CS3 DDS 3
FTW = ((freq2 * 4294967296)/1000000000)
#print(int(FTW))
s3 =int(FTW).to_bytes(4,'little')
#print(s3)
list_of_value1 =[CFR_ADRESS, CFR_1, CFR_2, CFR_3, CFR_4]
spi.xfer(list_of_value1)
list_of_value7 = [FTW0_ADRESS,s3[3],s3[2],s3[1],s3[0]]
spi.xfer(list_of_value7)
GPIO.setup(update_DDS, GPIO.OUT)  # update DDS AD9858
GPIO.output(update_DDS, True)
time.sleep(E_DELAY)
GPIO.output(update_DDS, False)
print("data fréquence F2 en mémoire récupéree")

#DDS3 AD9852 activation
GPIO.output(CS1, True) # CS1 DDS 1
GPIO.output(CS2, True) # CS2 DDS 2
GPIO.output(CS3, False) # CS2 DDS 3
FTW = ((freq3 * 281474976710656)/340000000)
#print(int(FTW))
s5 =int(FTW).to_bytes(6,'little')
#print(s5)
# PLL definition
list_of_value1 =[PLL_ADRESS, PLL_0, PLL_1, PLL_2, PLL_3]
spi.xfer(list_of_value1)
list_of_value8 = [FTW_ADRESS,s5[5],s5[4],s5[3],s5[2],s5[1],s5[0]]
spi.xfer(list_of_value8)
print("data fréquence F3 en mémoire récupéree")
s4 =amp3.to_bytes(2,'little')
print(amp3)
list_of_value5 = [OSK_ADRESS,s4[1],s4[0]]
spi.xfer2(list_of_value5)
print("data amplitude F3 en mémoire récupéree")

#DDS4 AD9852 activation
GPIO.output(CS1, True) # CS1 DDS 1
GPIO.output(CS2, True) # CS2 DDS 2
GPIO.output(CS3, True) # CS2 DDS 3
GPIO.output(CS4, False)# CS4 DDS 4
FTW = ((freq4 * 281474976710656)/340000000)
s5 =int(FTW).to_bytes(6,'little')
# PLL definition
list_of_value1 =[PLL_ADRESS, PLL_0, PLL_1, PLL_2, PLL_3]
spi.xfer(list_of_value1)
list_of_value8 = [FTW_ADRESS,s5[5],s5[4],s5[3],s5[2],s5[1],s5[0]]
spi.xfer(list_of_value8)
print("data fréquence F4 en mémoire récupéree")
s4 =amp4.to_bytes(2,'little')
print(amp4)
list_of_value5 = [OSK_ADRESS,s4[1],s4[0]]
spi.xfer2(list_of_value5)
print("data amplitude F4 en mémoire récupéree")


def main():
  # Main program block
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BOARD)       # Use BOARD GPIO numbers
  GPIO.setup(LCD_E, GPIO.OUT)  # E
  GPIO.setup(LCD_RS, GPIO.OUT) # RS
  GPIO.setup(LCD_D0, GPIO.OUT) # DB0
  GPIO.setup(LCD_D1, GPIO.OUT) # DB1
  GPIO.setup(LCD_D2, GPIO.OUT) # DB2
  GPIO.setup(LCD_D3, GPIO.OUT) # DB3
  GPIO.setup(LCD_D4, GPIO.OUT) # DB4
  GPIO.setup(LCD_D5, GPIO.OUT) # DB5
  GPIO.setup(LCD_D6, GPIO.OUT) # DB6
  GPIO.setup(LCD_D7, GPIO.OUT) # DB7
  # Initialise display
  lcd_init()
  lcd_clear()
  LCD_LINE_1 = 0x82
  LCD_LINE_2 = 0xC2
  LCD_LINE_3 = 0x94
  LCD_LINE_4 = 0xD2
  lcd_string("------------",LCD_LINE_1)
  lcd_string("Rasbperry Pi",LCD_LINE_2)
  lcd_string("Model B",LCD_LINE_3)
  lcd_string("------------",LCD_LINE_4)
  time.sleep(2)
  lcd_clear()
  LCD_LINE_1 = 0x80
  LCD_LINE_2 = 0xC0
  LCD_LINE_3 = 0x92
  LCD_LINE_4 = 0xD2
  lcd_string("Rack DDS AD9858",LCD_LINE_1)
  lcd_string("On Rasbperry Pi",LCD_LINE_2)
  lcd_string(" wait data",LCD_LINE_3)
  lcd_string("------------",LCD_LINE_4)
  lcd_clear()
  LCD_LINE_1 = 0x80
  LCD_LINE_2 = 0xC0
  LCD_LINE_3 = 0x92
  LCD_LINE_4 = 0xD0
  ipaddr = run_cmd(cmd)
  lcd_string("Rack DDS AD9858",LCD_LINE_1)
  lcd_string("On Rasbperry Pi",LCD_LINE_2)
  lcd_string(" IP adress:",LCD_LINE_3)
  lcd_string('%s'%ipaddr,LCD_LINE_4)
  

  while True:
             data = ser.read(7)
             print("données reçues de l'UART")
             data_amp = int.from_bytes([data[1],data[0]],"big")
             data_freq = data[5]<<23 + data[4]<<16 + data[3]<<8 + data[2]
             data_select_dds = data[6]
             print("DDS numéro =",data_select_dds)
             print(data_select_dds)
             unpacked = struct.unpack_from("L",data,2)
             print(unpacked)
           
                                        
             if data_select_dds ==1:
               GPIO.output(CS1, False)  # CS1 DDS 1
               GPIO.output(CS2, True) # CS2 DDS 2
               GPIO.output(CS3, True) # CS3 DDS 3
               GPIO.output(CS4, True)# CS4 DDS 4
               res = int(''.join(map(str,unpacked)))
               print(res)
               f = open("freq1_memory","wb")
               freq1 = res
               d = {"freq1": freq1}
               dump(d,f)
               f.close()
               # CFR register AD9858
               list_of_value1 =[CFR_ADRESS, CFR_1, CFR_2, CFR_3, CFR_4]
               spi.xfer(list_of_value1)
               time.sleep(0.01)
               FTW = ((freq1 * 4294967296)/1000000000)
               s1 =int(FTW).to_bytes(4,'little')
               list_of_value6 = [FTW0_ADRESS,s1[3],s1[2],s1[1],s1[0]]
               time.sleep(0.01)
               spi.xfer(list_of_value6)
               print("data fréquence F1 envoyée")
               print("freq1 =",freq1)
               lcd_clear()
               LCD_LINE_1 = 0x81
               LCD_LINE_2 = 0xC4
               LCD_LINE_3 = 0x90
               LCD_LINE_4 = 0xD3
               lcd_string("Chanel(1) in Hz ",LCD_LINE_1)
               lcd_string("%s"%freq1,LCD_LINE_2)
               #lcd_string(" Amp(1) (0-4095)",LCD_LINE_3)
               lcd_string(" Fclock = 1GHz",LCD_LINE_3)
               lcd_string("--AD9858--",LCD_LINE_4)
               GPIO.setup(update_DDS, GPIO.OUT)  # update DDS AD9858
               GPIO.output(update_DDS, True)
               time.sleep(0.01)
               GPIO.output(update_DDS, False)
                            
              
             if data_select_dds ==2:
               GPIO.output(CS1, True)  # CS1 DDS 1
               GPIO.output(CS2, False) # CS2 DDS 2
               GPIO.output(CS3, True) # CS3 DDS 3
               GPIO.output(CS4, True)# CS4 DDS 4
               res = int(''.join(map(str,unpacked)))
               print(res)
               f = open("freq2_memory","wb")
               freq2 = res
               d = {"freq2": freq2}
               dump(d,f)
               f.close()
               FTW = ((freq2 * 4294967296)/1000000000)
               # CFR register AD9858
               list_of_value1 =[CFR_ADRESS, CFR_1, CFR_2, CFR_3, CFR_4]
               spi.xfer(list_of_value1)
               s3 =int(FTW).to_bytes(4,'little')
               list_of_value6 = [FTW0_ADRESS,s3[3],s3[2],s3[1],s3[0]]
               spi.xfer(list_of_value6)
               print("data fréquence F2 envoyée")
               print("freq2 =",freq2)
               lcd_clear()
               LCD_LINE_1 = 0x81
               LCD_LINE_2 = 0xC4
               LCD_LINE_3 = 0x90
               LCD_LINE_4 = 0xD3
               lcd_string("Chanel(2) in Hz ",LCD_LINE_1)
               lcd_string("%s"%freq2,LCD_LINE_2)
               lcd_string(" Fclock = 1GHz",LCD_LINE_3)
               lcd_string("--AD9858--",LCD_LINE_4)
               GPIO.setup(update_DDS, GPIO.OUT)  # update DDS AD9858
               GPIO.output(update_DDS, True)
               time.sleep(0.01)
               GPIO.output(update_DDS, False)
     
               
             if data_select_dds ==3:
               GPIO.output(CS1, True) # CS1 DDS 1
               GPIO.output(CS2, True) # CS2 DDS 2
               GPIO.output(CS3, False) # CS3 DDS 3
               GPIO.output(CS4, True)# CS4 DDS 4
               res = int(''.join(map(str,unpacked)))
               print(res)
               f = open("amp3_memory","wb")
               amp3 = data_amp
               d = {"amp3": amp3}
               dump(d,f)
               f.close()
               s4 =amp3.to_bytes(2,'little')
               list_of_value5 = [OSK_ADRESS,s4[1],s4[0]]
               spi.xfer2(list_of_value5)
               print("data amplitude F3 envoyée")
               print("amplitude=",amp3)
               f = open("freq3_memory","wb")
               freq3 = res
               d = {"freq3": freq3}
               dump(d,f)
               f.close()
               # PLL definition
               list_of_value1 =[PLL_ADRESS, PLL_0, PLL_1, PLL_2, PLL_3]
               spi.xfer(list_of_value1)
               FTW = ((freq3 * 281474976710656)/340000000)
               s5 =int(FTW).to_bytes(6,'little')
               list_of_value6 = [FTW_ADRESS,s5[5],s5[4],s5[3],s5[2],s5[1],s5[0]]
               spi.xfer(list_of_value6)
               print("data fréquence F3 envoyée")
               print("freq3 =",freq3)
               lcd_clear()
               LCD_LINE_1 = 0x81
               LCD_LINE_2 = 0xC4
               LCD_LINE_3 = 0x90
               LCD_LINE_4 = 0xD4
               lcd_string("Chanel(3) in Hz ",LCD_LINE_1)
               lcd_string("%s"%freq3,LCD_LINE_2)
               lcd_string(" AD9852 (0-4095)",LCD_LINE_3)
               lcd_string("Amp=%s"   %amp3,LCD_LINE_4)
            
             if data_select_dds ==4:
                GPIO.output(CS1, True) # CS1 DDS 1
                GPIO.output(CS2, True) # CS2 DDS 2
                GPIO.output(CS3, True) # CS3 DDS 3
                GPIO.output(CS4, False)# CS4 DDS 4
                res = int(''.join(map(str,unpacked)))
                print(res)
                f = open("amp4_memory","wb")
                amp4 = data_amp
                d = {"amp4": amp4}
                dump(d,f)
                f.close()
                s8 =amp4.to_bytes(2,'little')
                list_of_value5 = [OSK_ADRESS,s8[1],s8[0]]
                spi.xfer2(list_of_value5)
                print("data amplitude F4 envoyée")
                print("amplitude=",amp4)
                f = open("freq4_memory","wb")
                freq4 = res
                d = {"freq4": freq4}
                dump(d,f)
                f.close()
                # PLL definition
                list_of_value1 =[PLL_ADRESS, PLL_0, PLL_1, PLL_2, PLL_3]
                spi.xfer(list_of_value1)
                FTW = ((freq4 * 281474976710656)/340000000)
                s5 =int(FTW).to_bytes(6,'little')
                list_of_value6 = [FTW_ADRESS,s5[5],s5[4],s5[3],s5[2],s5[1],s5[0]]
                spi.xfer(list_of_value6)
                print("data fréquence F4 envoyée")
                print("freq4 =",freq4)
                lcd_clear()
                LCD_LINE_1 = 0x81
                LCD_LINE_2 = 0xC4
                LCD_LINE_3 = 0x90
                LCD_LINE_4 = 0xD4
                lcd_string("Chanel(4) in Hz ",LCD_LINE_1)
                lcd_string("%s"%freq4,LCD_LINE_2)
                lcd_string(" AD9852 (0-4095)",LCD_LINE_3)
                lcd_string("Amp=%s"   %amp4,LCD_LINE_4)
 
def lcd_init():
  # Initialise display
  #lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  #lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x38,LCD_CMD) # 101000 Data length 8 bits, number of lines =2, font size =5x7 dots
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction

  
def lcd_clear():
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
  
 
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command
    
  GPIO.output(LCD_RS, mode) # RS
  # High bits
  GPIO.output(LCD_D0, False)
  GPIO.output(LCD_D1, False)
  GPIO.output(LCD_D2, False)
  GPIO.output(LCD_D3, False)
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  
  if bits&0x01==0x01:
    GPIO.output(LCD_D0, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D1, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D2, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D3, True)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)
  # Toggle 'Enable' pin
  lcd_toggle_enable()
    
def lcd_string(message,line):
  # Send string to display 
  message = message.ljust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)
 
if __name__ == '__main__':
    
    
 try:
   main()
 except KeyboardInterrupt:
     pass
 finally:
       lcd_byte(0x01,LCD_CMD) # 000001 Clear display
       print("Fin du programme")
       GPIO.cleanup()
  



