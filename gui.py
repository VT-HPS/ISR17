#!/usr/bin/python3
import re
import os
import time
import serial
from tkinter import *
from tkinter import ttk
from math import sqrt, pow
from numpy import interp
import threading
import random

#USB = '/dev/ttyACM0'        #raspberry pi - check which usb port the arduino is hooked to
USB = '/dev/cu.usbmodem11301' #testing computer - check which usb port the arduino is hooked to
print("hit initial")
gyro_list = [0,0]
depth = 0
psv = 0
ps_value = 0
rpm_value = 0 
rpm_graphic_coord = 0
depth_graphic_coord = 0
deptg_indicator = 0


######### CREATE DATALOGGING FILE ###################################################################################

home_dir = os.path.expanduser('~') # Get the path to the user's home directory
file_path = os.path.join(home_dir, 'ISR17/ISR17/serial_list_data.txt') # Create a file path in the home directory
print(file_path)

######### CONNECT WITH ARDUINO ######################################################################################

arduino = serial.Serial(USB, 115200, timeout=0)

def read_arduino():
    return arduino.readline()[:-2] #the last bit gets rid of the new-line chars


######### DASHBOARD DISPLAY ITEMS ######################################################################################

# MINOR DISPLAY SETUP ITEMS
root = Tk()  # initialize root variable
root.geometry("800x480")  # root sized to hdmi monitor
root.title('Nautilus HUD')

# STYLE CONFIGURATION
root.style = ttk.Style(root)
root.style.configure('title.TLabel', font=('Times', 14, 'bold')) # For gauge labels
root.style.configure('.TLabel', font=('Times', 14)) # gauge readings

# GRID MANAGEMENT (2x3)
root.columnconfigure(0, weight=3)
root.columnconfigure(1, weight=1)
root.rowconfigure(1, weight=3)
root.rowconfigure(2, weight=2)

# HEADING DISPLAY SETUP
HEIGHT = 200
WIDTH = 200
RADIUS = 30
TAG = "cir"
# labels
heading_label = ttk.Label(root, text='HEADING', style='title.TLabel').grid(column=0, row=0, sticky='n')
heading_up = ttk.Label(root, text='UP', style='.TLabel').grid(column=0, row=1, sticky='n')
heading_port = ttk.Label(root, text='PORT', style='.TLabel').grid(column=0, row=1, sticky='w', padx=100)
heading_starboard = ttk.Label(root, text='STARBOARD', style='.TLabel').grid(column=0, row=1, sticky='e', padx=40)
heading_down = ttk.Label(root, text='DOWN', style='.TLabel').grid(column=0, row=1, sticky='s')
# canvas items
heading_canvas = Canvas(root, width=WIDTH, height=HEIGHT)
heading_canvas.create_line((WIDTH/2)+20, (HEIGHT/2), (HEIGHT/2)-20, (HEIGHT/2))
heading_canvas.create_line((WIDTH/2), (HEIGHT/2)+20, (HEIGHT/2), (HEIGHT/2)-20)
heading_canvas.create_oval((WIDTH/2)-RADIUS, (HEIGHT/2)-RADIUS, (WIDTH/2)+RADIUS,
        (HEIGHT/2)+RADIUS, fill='green', tags=TAG)
heading_canvas.grid(column=0, row=1)

# DEPTH DISPLAY SETUP
# labels
depth_label = ttk.Label(root, text='DEPTH', style='title.TLabel').grid(column=1, row=0, sticky='n')
depth_value_label = ttk.Label(root, text="0", style='.TLabel')
depth_value_label.place(x=730, y=25, anchor='n')
# canvas items
depth_canvas = Canvas(root, height=400, width=100)
depth_canvas.create_rectangle(3, 400, 100, 3, width='3')
depth_canvas.grid(column=1, row=1, rowspan=2)
# (x0, y0, x1, y1) = (over 3, down 400, over 100, go down to depth #)
depth_bar = depth_canvas.create_rectangle(3, 400, 100, depth, fill='#FFCC00')


# RPM DISPlAY SETUP
#labels
RPM_label = ttk.Label(root, text='RPM', style='title.TLabel').grid(column=0, row=2, sticky="nw")
RPM_value_label = ttk.Label(root, text="0", style='.TLabel')
RPM_value_label.place(x=20, y=410, anchor='w')
# canvas items
RPM_canvas = Canvas(root, height=100, width=550)
RPM_canvas.create_rectangle(550, 3, 3, 100, width='3')
RPM_bar = RPM_canvas.create_rectangle(rpm_value, 3, 3, 100, fill='#FFCC00')
RPM_canvas.grid(column=0, row=2)


# change color of circle on display according to value
def get_circle_color(x, y, radius):
    #check if coordinate is with in radius of origin
    if  radius > int(sqrt( pow(abs(x-(WIDTH/2)), 2) + pow(abs(y-(HEIGHT/2)), 2))): 
        return 'green'
    elif radius * 2 < int(sqrt( pow(abs(x-(WIDTH/2)), 2) + pow(abs(y-(HEIGHT/2)), 2))): 
        return 'red'
    else:
        return 'yellow'


# create circle element for display
def create_circle(x, y, r, canvasName, t):
    color = get_circle_color(x,y,r)
    canvasName.create_oval(x-r, y-r, x+r, y+r, fill=color, tags=t)
    root.update()


# delete circle element for display
def delete_circle(canvasName, tag):
    canvasName.delete(tag)
    root.update()



######### INTERPRET PRESSURE SENSOR ######################################################################################

# convert pressure sensor voltage to coordinates for canvas display
def convert_volts_to_coord(psv_data):
    #print('psv_data = ', psv_data)
    depth_feet = round(psv_data/12)
    print('depth_feet ', depth_feet)
    if depth_feet >= 30:
        return 3
    elif depth_feet <= 0:
        return 400
    else:
        # print('psv_data ', psv_data)
        # change these voltages for the pool voltages 0(0.5V) - 1023(4.5V)
        depth_indicator = interp(depth_feet,[0,30],[400,3])
    return int(depth_indicator)


# calculate depth based on pressure 
def calculate_depth(psv_data):
    depth_feet = round(psv_data/12)
    print("Depth Value (m) = ", depth_feet)
    return depth_feet



######### INTERPRET RPM SENSOR ######################################################################################

# convert rpm value to coordinate for display
def convert_rpms_to_coord(data):
    if data >= 250:
        return 550
    elif data <= 0:
        return 3
    else:
        # change these rpm ranges for estimated RPM range
        rpm_indicator = interp(data,[0,250],[3,550])
        return int(abs(rpm_indicator))


######### INTERPRET GYRO SENSOR ######################################################################################

# map incoming gryo bytes to 1 - 9 
def filter_gyro_coord(data): 
    if data[2] > 90.0:
        data[2] = 90
    elif data[2] < -90:
        data[2] = -90
    else:
        data[2] = data[2]

    if data[3] > 90.0:
        data[3] = 90
    elif data[3] < -90:
        data[3] = -90
    else:
        data[3] = data[3]
    return data 


# convert mapped gyro data to coordinates for display
def convert_gyro_to_coord(data):
    print('convert_gyro_to_coord(data): ', data)
    y = interp(int(data[2]/10),[-9,9],[150,50])
    x = interp(int(data[3]/10),[-9,9],[50,150])
    gyro_list = [int(x), int(y)]
    return gyro_list


######### DISPLAY AND UPDATE DASHBOARD ITEMS ######################################################################################

def gyro_dashboard():
    delete_circle(heading_canvas, TAG)
    create_circle(gyro_list[0], gyro_list[1], RADIUS, heading_canvas, TAG)


def rpm_dashboard():
    rpm_graphic_coord = convert_rpms_to_coord(rpm_value)
    RPM_canvas.coords(RPM_bar, rpm_graphic_coord, 3, 3, 100)
    print('rpm_value = ', rpm_value)
    RPM_value_label.config(text=str(rpm_value))
    


def depth_dashboard():
    depth = calculate_depth(ps_value)
    depth_graphic_coord = convert_volts_to_coord(ps_value)
    depth_canvas.coords(depth_bar, 3, 400, 100, depth_graphic_coord)
    depth_value_label.config(text=str(depth))


def update_gui():
    while True:
        print('new update ---------------------------------------------------')
        gyro_dashboard()
        rpm_dashboard()
        depth_dashboard()
        time.sleep(0.1)


def close_win():
   root.destroy()


# FOR TESTING
def get_random_xy_coord():
    global gyro_coord
    global rpm_value
    global decode_rpms
    global depth_indicator
    global decode_ps_voltage
    global ps_value
    global rpm_graphic_coord

    # CREATE A FILE FOR OUTPUTING SERIAL DATA FOR DATALOGGING
    with open(file_path, 'w') as output_file:
        while True:
            time.sleep(0.5)
            data = [random.randrange(-9,9,1), random.randrange(-9,9,1)]
            y = interp(int(data[0]/10),[-9,9],[150,50])
            x = interp(int(data[1]/10),[-9,9],[50,150])
            gyro_coord = [int(x), int(y)]

            rpm_value = random.randrange(0, 250)
            #rpm_value = 100
            print('rpm_value = ', rpm_value)
            

            ps_value = random.randrange(12, 360)
            depth_indicator = convert_volts_to_coord(ps_value)
            serial_list = [depth_indicator, rpm_value, int(x), int(y)]
            print('SERIAL LIST ', str(serial_list))
            
            # OUTPUT SERIAL DATA FOR DL
            output_file.write(str(serial_list) + "\n")
            output_file.flush()    
# END OF FOR TESTING



def read_sensor_data():
    global ps_value
    global rpm_value
    global depth_graphic_coord
    global rpm_graphic_coord
    global gyro_list
    serial_list = []
    serial_list_backup = [0,0,0,0,0,0]

    # CREATE A FILE FOR OUTPUTING SERIAL DATA FOR DATALOGGING
    with open(file_path, 'w') as output_file:
        while True:
            serial_list = []
            data = read_arduino()
            print('--------------------------------------------')
            print('data: ', data)        
            try: 
                serial_string = (data.decode('ascii'))
            except:
                serial_string = '' 
            #serial_string = serial_string.replace('\r', '')
            serial_string = serial_string.strip()
            print('serial string strip!!')
            print(serial_string)
            #serial_string = re.sub('[^\d,.-]|[.-](?=[^.]*[.])', '', serial_string) 
            serial_string_split = serial_string.split(",")
            print('serial string split = ', serial_string_split)

            if (data == b'') or (re.match( r'^\.' or r'^\>' or '^\!', serial_string)) :
                print('data2 ', data)
                serial_list = serial_list_backup.copy()
            else:    
                if "!" not in serial_string.split(","):
                    for x in serial_string.split(","):
                        if x != '': 
                            try:
                                serial_list.append(float(x)) 
                            except:
                                serial_list.append(0)
                        else:
                            serial_list.append(0)
                    print("Original Serial List: ", serial_list)
                    if len(serial_list) != 6 and len(serial_list) != 5:
                        serial_list = [0,0,0,0,0,0]
                    print('serial data list: ', serial_list)
                    serial_list_backup = serial_list.copy()
                    print('SERIAL LIST BU', serial_list_backup)
                else:
                    serial_list = serial_list_backup.copy()
                    print('HIT BACK UP SERIAL LIST')
                     
                
            filtered_gyro_values = filter_gyro_coord(serial_list) # make sure gyro data is between -90 and 90
            print('serial list with filtered gyro values: ', filtered_gyro_values) 
            print('')

            gyro_list = convert_gyro_to_coord(filtered_gyro_values)
            print('gyro_list: ', gyro_list, '- mapped from [-9,9],[50,150] and [-9,9],[150,50]') 

            ps_value = serial_list[0]
            depth_graphic_coord = convert_volts_to_coord(ps_value)
            print('ps_coord: ', depth_graphic_coord, ' - mapped from [0,1023] to [400,100]') 

            rpm_value = serial_list[1]
            rpm_graphic_coord = convert_rpms_to_coord(rpm_value)
            print('rpm_graphic_coord: ', rpm_graphic_coord, '- mapped from [0,250] to [3,550]') 
            print('SERIAL LIST = ', serial_list)

            control_surface_degrees = serial_list[4:]
            #print("Pitch degrees: ", control_surface_degrees[0], ", Yaw degrees: ", control_surface_degrees[1])

            # OUTPUT SERIAL DATA FOR DL
            output_file.write(str(serial_list) + "\n")
            output_file.flush()  
       

if '__main__' == __name__:
    # heading elements

    # random data testing
    #th = threading.Thread(target=get_random_xy_coord, args=(),  daemon=True)

    th = threading.Thread(target=read_sensor_data, args=(),  daemon=True)
    th.start()
    
    update_gui()

    root.mainloop()  # run hud
    #test.mainloop()
