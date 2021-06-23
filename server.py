import socket
import os
import pyzbar.pyzbar as pyzbar
import cv2

import RPi.GPIO as GPIO #control motor board through GPIO pins
import time #set delay time to control moving distance

IR1 = 22
IR2 = 24
IR3 = 26
IR15 = 7
IR25 = 11

#If IN1Rear=True and IN2Rear=False left motor move forward 
#If IN1Rear=False,IN2Rear=True left motor move backward,in other cases right motor stop
IN1Rear = 16 #GPIO23 to IN1 Rear-right wheel direction 
IN2Rear = 18 #GPIO24 to IN2 Rear-right wheel direction

#If IN3Rear=True and IN3Rear=False left motor move forward
#If IN3Rear=False,IN4Rear=True left motor move backward,in other cases left motor stop
IN3Rear = 13 #GPIO27 to IN3 Rear-left wheel direction
IN4Rear = 15 #GPIO22 to IN4 Rear-left wheel direction

# PWM Pin for PWM 
ENA = 33 

#If IN1Front=True and IN2Front=False right motor move forward, If IN1Front=False,IN2Front=True right motor move backward,in other cases right motor stop
IN1Front = 40
IN2Front = 38

#If IN3Front=True and IN3Front=False left motor move forward
#If IN3Front=False,IN4Front=True left motor move backward,in other cases left motor stop
IN3Front = 36
IN4Front = 32

DIR = 0 # direction initialize

GPIO.setmode(GPIO.BOARD)
GPIO.setup(IR1, GPIO.IN)
GPIO.setup(IR2, GPIO.IN)
GPIO.setup(IR3, GPIO.IN)
GPIO.setup(IR15, GPIO.IN)
GPIO.setup(IR25, GPIO.IN)

#initialize GPIO pins, tell OS which pins will be used to control Model-Pi L298N board
GPIO.setup(IN1Rear, GPIO.OUT) 
GPIO.setup(IN2Rear, GPIO.OUT)
GPIO.setup(IN3Rear, GPIO.OUT)
GPIO.setup(IN4Rear, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1Front, GPIO.OUT) 
GPIO.setup(IN2Front, GPIO.OUT)
GPIO.setup(IN3Front, GPIO.OUT)
GPIO.setup(IN4Front, GPIO.OUT)
GPIO.output(IN1Rear, False)
GPIO.output(IN2Rear, False)
GPIO.output(IN3Rear, False)
GPIO.output(IN4Rear, False)
GPIO.output(IN1Front, False)
GPIO.output(IN2Front, False)
GPIO.output(IN3Front, False)
GPIO.output(IN4Front, False)

# PWM Pin ENA
rspeed = GPIO.PWM(ENA,20)	

rspeed.start(0)

#make rear right motor moving forward
def rl_ahead(speed):
    GPIO.output(IN1Rear,True)  # 16
    GPIO.output(IN2Rear,False) # 18
    #ChangeDutyCycle(speed) function can change the motor rotation speed
    rspeed.ChangeDutyCycle(speed) #33
    
#make rear left motor moving forward    
def rr_ahead(speed):  
    GPIO.output(IN3Rear,True)  # 13
    GPIO.output(IN4Rear,False) # 15
    rspeed.ChangeDutyCycle(speed) # 12
    
#make rear right motor moving backward
def rl_back(speed):
    GPIO.output(IN2Rear,True)
    GPIO.output(IN1Rear,False)
    rspeed.ChangeDutyCycle(speed)

#make rear left motor moving backward    
def rr_back(speed):  
    GPIO.output(IN4Rear,True)
    GPIO.output(IN3Rear,False)
    rspeed.ChangeDutyCycle(speed)    
    
#make front right motor moving forward
def fr_ahead(speed):
    GPIO.output(IN1Front,True)    # 40 
    GPIO.output(IN2Front,False)   # 38
    rspeed.ChangeDutyCycle(speed)  # 3
    
#make Front left motor moving forward    
def fl_ahead(speed):  
    GPIO.output(IN3Front,True)    # 36 
    GPIO.output(IN4Front,False)   # 32
    rspeed.ChangeDutyCycle(speed)  # 5
    
#make Front right motor moving backward
def fr_back(speed):
    GPIO.output(IN2Front,True)
    GPIO.output(IN1Front,False)
    rspeed.ChangeDutyCycle(speed)

#make Front left motor moving backward    
def fl_back(speed):  
    GPIO.output(IN4Front,True)
    GPIO.output(IN3Front,False)
    rspeed.ChangeDutyCycle(speed)

def turning_left(speed,time_t):
    fr_ahead(speed)
    fl_back(speed)
    rr_ahead(speed)
    rl_back(speed)
    time.sleep(time_t)

def turning_right(speed,time_t):
    fr_back(speed)
    fl_ahead(speed)
    rr_back(speed)
    rl_ahead(speed)
    time.sleep(time_t)
    
def go_ahead(speed):
    rl_ahead(speed)
    rr_ahead(speed)
    fl_ahead(speed)
    fr_ahead(speed)

def turn_right(speed):
    fr_back(speed)
    fl_ahead(speed)
    rr_back(speed)
    rl_ahead(speed)
def turn_left(speed):
    fr_ahead(speed)
    fl_back(speed)
    rr_ahead(speed)
    rl_back(speed)

#make both motor stop
def stop_car():
    GPIO.output(IN1Rear,False)
    GPIO.output(IN2Rear,False)
    GPIO.output(IN3Rear,False)
    GPIO.output(IN4Rear,False)
    GPIO.output(IN1Front,False)
    GPIO.output(IN2Front,False)
    GPIO.output(IN3Front,False)
    GPIO.output(IN4Front,False)
    rspeed.ChangeDutyCycle(0)

def line_tracing():
    while True :
        right_Sensor = GPIO.input(IR1)
        lf_Sensor = GPIO.input(IR15)
        front_Sensor = GPIO.input(IR2)
        rf_Sensor = GPIO.input(IR25)
        left_Sensor = GPIO.input(IR3)
        Sensor = str(left_Sensor) + str(lf_Sensor) + str(front_Sensor) + str(rf_Sensor) + str(right_Sensor)    
        if( Sensor == "00111"):
            stop_car()
            break
        elif ( Sensor == "00100" or Sensor == "00110" or Sensor == "01100" ):
            go_ahead(25)
            time.sleep(0.1)
        elif ( Sensor == "00010" or Sensor == "00001" ):
            while True:
                right_Sensor = GPIO.input(IR1)
                lf_Sensor = GPIO.input(IR15)
                front_Sensor = GPIO.input(IR2)
                rf_Sensor = GPIO.input(IR25)
                left_Sensor = GPIO.input(IR3)
                Sensor = str(left_Sensor) + str(lf_Sensor) + str(front_Sensor) + str(rf_Sensor) + str(right_Sensor)
                if ( front_Sensor == 1 or left_Sensor == 1) :
                    break
                turn_right(25)
                time.sleep(0.001)
                print(Sensor)
        elif( Sensor == "01000" or Sensor == "10000" ):
            while True:
                right_Sensor = GPIO.input(IR1)
                lf_Sensor = GPIO.input(IR15)
                front_Sensor = GPIO.input(IR2)
                rf_Sensor = GPIO.input(IR25)
                left_Sensor = GPIO.input(IR3)
                Sensor = str(left_Sensor) + str(lf_Sensor) + str(front_Sensor) + str(rf_Sensor) + str(right_Sensor)
                if ( front_Sensor == 1 or right_Sensor == 1) :
                    break
                turn_left(25)
                time.sleep(0.001)
                print(Sensor)
        elif (Sensor == "00100" or Sensor == "01010"):
            go_ahead(25)
            time.sleep(0.001)
        else:
            time.sleep(0.001)
        print(Sensor)	
def readpath(data):
    data = data.decode()
    data = data.split('\n')
    for i in range(len(data)-1):
        data[i] = data[i].replace(',', ' ').replace(')', '').replace('(','')
    data.remove('')
    return data

def qrread():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1024)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,768)
    i = 0
    while(cap.isOpened()):
      ret, img = cap.read()
      if not ret:
        continue

      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         
      decoded = pyzbar.decode(gray)

      for d in decoded: 
        x, y, w, h = d.rect

        barcode_data = d.data.decode("utf-8")
        barcode_type = d.type
        
        if barcode_data != '0':
            path_name = './start_loc.txt'
            with open(path_name, "w+") as lf:
                lf.write(barcode_data)
                lf.close()
            cap.release()
            cv2.destroyAllWindows()
            return 
        
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
        text = '%s (%s)' % (barcode_data, barcode_type)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

      cv2.imshow('img', img)
        
      key = cv2.waitKey(1)
      if key == ord('q'):
        break
      elif key == ord('s'):
        i += 1
        cv2.imwrite('c_%03d.jpg' % i, img)

    cap.release()
    cv2.destroyAllWindows()

def read_startloc():
    f = open('start_loc.txt', 'r')
    s = f.read()
    f.close()
    return s

def turning(curx, cury, x, y, DIR):
    diffx = x-curx
    diffy = y-cury

    next_dir = 0
    if diffx == 0 and diffy == -1:
        next_dir = 3  # E
    elif diffx == -1 and diffy == 0:
        next_dir = 2  # S
    elif diffx == 0 and diffy == 1:
        next_dir = 1  # W
    elif diffx == 1 and diffy == 0:
        next_dir = 0  # N

    Rotate = (next_dir - DIR if next_dir - DIR >= 0 else next_dir - DIR + 4)
    if Rotate < 3:
        for i in range(Rotate):
            turning_left(80,0.7)
            stop_car()
            time.sleep(0.2)
    else :
        turning_right(70,0.7)
        stop_car()
        time.sleep(0.2)
    return next_dir

gw = os.popen("ip -4 route show default").read().split()
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((gw[2], 0))
HOST = s.getsockname()[0]
print(HOST)
s.close()
PORT = 9999

# socket crate
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR, 1)
server_socket.bind((HOST,PORT))

while True:
    server_socket.listen()
    client_socket, addr = server_socket.accept()
    print('Connected by', addr)
    copy = '0'
    k = client_socket.recv(1024)
    
    print('Received from', addr, k.decode())
    # start location read
    qrread()    
    s = read_startloc()
    if copy == s: # if not search start location
        continue
    else :
        print('start location :',s) # start location search -> send all
        copy = s
        client_socket.sendall(s.encode())

    s = s.split()
    
    curx = int(s[0])
    cury = int(s[1]) # start location

    data = client_socket.recv(1024)
    path = readpath(data)
    
    while path :
        
        next_loc = path[0].split()
        
        x = int(next_loc[0])
        y = int(next_loc[1])

        if (x == curx ) and (y == cury):
            del path[0]
            continue
        DIR = turning(curx, cury, x, y, DIR)
        go_ahead(20)
        time.sleep(0.8)
        
        line_tracing()
        qrread()
        s = read_startloc()
        if (s.split() != path[0].split()):
            if(len(data) == 0):
                break
            client_socket.sendall(s.encode())
            s = s.split()
            curx = int(s[0])
            cury = int(s[1]) # start location
            data = client_socket.recv(1024)
            path = readpath(data)
            continue
        curx = x
        cury = y
        del path[0]
    s = '1'
    client_socket.sendall(s.encode())   
    client_socket.close()

GPIO.cleanup()
server_socket.close()
