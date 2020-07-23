#IMPORT THE PACKAGES
import numpy as np
import cv2
import RPi.GPIO as gpio
import numpy as np
import time
import serial
import os
from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

#### INITIALIZE THE MOTOR PINS
def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) #IN1
    gpio.setup(33, gpio.OUT) #IN2
    gpio.setup(35, gpio.OUT) #IN3
    gpio.setup(37, gpio.OUT) #IN4
    gpio.setup(36, gpio.OUT)
    gpio.setup(7 , gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(12 , gpio.IN, pull_up_down = gpio.PUD_UP)
init()

#### SET THE GRIPPER WITH THE PWM VALUES
pwmservo = gpio.PWM(36,50)
pwmservo.start(12)

#### DEFINE ULTRASONIC SENSOR PINS & RECORD THE DISTANCE FROM THE SENSORS 
trig = 16
echo = 18
def distance():
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig,gpio.OUT)
    gpio.setup(echo,gpio.IN)
    gpio.output(trig, False)
    time.sleep(0.10)
    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)
    while gpio.input(echo) == 0:
        pulse_start = time.time()
    while gpio.input(echo) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration*17150
    distance = round(distance,2)
    gpio.cleanup()
    return distance

#### STOP FUNCTION    
def gameover():
    init()
    #set all pins low
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)

#### FORWARD FUNCTION
def forward():    
    init()
    counterBR = np.uint64(0)
    buttonBR = int(0)
    counterFL = np.uint64(0)
    buttonFL = int(0)
    pwm2 = gpio.PWM(31,50)
    pwm = gpio.PWM(37,50)
    val = 20
    ticks = 5
    pwm2.start(val)
    pwm.start(val)
    time.sleep(0.1)
    e1_prev = 0
    e2_prev = 0
    kp = 0.1
    kd = 0.1
    ki = 0.02
    ts = 0.9
    e_prev_error = 0
    e2_prev_error = 0
    e_sum_error = 0
    e2_sum_error = 0
    for i in range(0,100000):
        print("counterFL = ",counterFL, "Gpio FL = ",gpio.input(7),"counter BR= ",counterBR, "Gpio BR= ",gpio.input(12))
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        e_error = counterBR - counterFL
        time.sleep(0.1)
        e_prev_error = e_error   
        val += (e_error * kp) + (e_prev_error * kd) + (e_sum_error * ki)
        val = max(min(20,val),0)
        pwm2.ChangeDutyCycle(val)
        pwm.ChangeDutyCycle(val)
        e_sum_error += e_error
        if  counterFL >= ticks:
            pwm2.stop()
        if  counterBR >= ticks:
            pwm.stop()
        if  counterFL >= ticks and counterBR >= ticks:
            gameover()
            print("Thanks for playing !")
            break

#### LEFT FUNCTION
def left(timeL):    
    init()
    counterBR = np.uint64(0)
    buttonBR = int(0)
    counterFL = np.uint64(0)
    buttonFL = int(0)
    pwm2 = gpio.PWM(33,50)
    pwm = gpio.PWM(37,50)
    val = 25
    pwm2.start(val)
    pwm.start(val)
    time.sleep(0.1)
    e1_prev = 0
    e2_prev = 0
    kp = 0.1
    kd = 0.1
    ki = 0.02
    ts = 0.9
    e_prev_error = 0
    e2_prev_error = 0
    e_sum_error = 0
    e2_sum_error = 0
    t = time.time()
    while time.time()-t < timeL:
        print("counterFL = ",counterFL, "Gpio FL = ",gpio.input(7),"counter BR= ",counterBR, "Gpio BR= ",gpio.input(12))
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        e_error = counterBR - counterFL
        time.sleep(0.1)
        e_prev_error = e_error   
        val += (e_error * kp) + (e_prev_error * kd) + (e_sum_error * ki)
        val = max(min(25,val),0)
        pwm2.ChangeDutyCycle(val)
        pwm.ChangeDutyCycle(val)
        e_sum_error += e_error 

#### RIGHT FUNCTION
def right(timeR):    
    init()
    counterBR = np.uint64(0)
    buttonBR = int(0)
    counterFL = np.uint64(0)
    buttonFL = int(0)
    pwm2 = gpio.PWM(31,50)
    pwm = gpio.PWM(35,50)
    val = 25
    pwm2.start(val)
    pwm.start(val)
    time.sleep(0.1)
    e1_prev = 0
    e2_prev = 0
    kp = 0.1
    kd = 0.1
    ki = 0.02
    ts = 0.9
    e_prev_error = 0
    e2_prev_error = 0
    e_sum_error = 0
    e2_sum_error = 0
    t = time.time()
    while time.time()-t < timeR:
        print("counterFL = ",counterFL, "Gpio FL = ",gpio.input(7),"counter BR= ",counterBR, "Gpio BR= ",gpio.input(12))
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        e_error = counterBR - counterFL
        time.sleep(0.1)
        e_prev_error = e_error   
        val += (e_error * kp) + (e_prev_error * kd) + (e_sum_error * ki)
        val = max(min(25,val),0)
        pwm2.ChangeDutyCycle(val)
        pwm.ChangeDutyCycle(val)
        e_sum_error += e_error

#### REVERSE FUNCTION
def reverse():    
    init()
    counterBR = np.uint64(0)
    buttonBR = int(0)
    counterFL = np.uint64(0)
    buttonFL = int(0)
    pwm2 = gpio.PWM(33,50)
    pwm = gpio.PWM(35,50)
    val = 25
    ticks = 17
    pwm2.start(val)
    pwm.start(val)
    time.sleep(0.1)
    e1_prev = 0
    e2_prev = 0
    kp = 0.1
    kd = 0.1
    ki = 0.2
    ts = 0.9
    e_prev_error = 0
    e2_prev_error = 0
    e_sum_error = 0
    e2_sum_error = 0
    for i in range(0,100000):
        print("counterFL = ",counterFL, "Gpio FL = ",gpio.input(7),"counter BR= ",counterBR, "Gpio BR= ",gpio.input(12))
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        e_error = counterBR - counterFL
        time.sleep(0.1)
        e_prev_error = e_error   
        val += (e_error * kp) + (e_prev_error * kd) + (e_sum_error * ki)
        val = max(min(25,val),0)
        pwm2.ChangeDutyCycle(val)
        pwm.ChangeDutyCycle(val)
        e_sum_error += e_error
        if  counterFL >= ticks:
            pwm2.stop()
        if  counterBR >= ticks:
            pwm.stop()
        if  counterFL >= ticks and counterBR >= ticks:
            gameover()
            print("Thanks for playing !")
            break

#### GRIPPER OPEN FUNCTION        
def pwm_open(tf):
    pwmservo.ChangeDutyCycle(12)
    time.sleep(1)    

#### GRIPPER CLOSE FUNCTION
def pwm_close(tf):
    pwmservo.ChangeDutyCycle(7)
    time.sleep(1)

#### GET THE INPUT 
cap = cv2.VideoCapture(0)
_, frame = cap.read()
rows, cols, _ = frame.shape
x_medium = int(cols / 2)
center1 = int(cols / 1.78) # CENTRE LINE FOR THE VIDEO


while True:
    _, frame = cap.read()

#### FLIP THE VIDEO SINCE CAMERA IS INVERTED
    frame = cv2.flip(frame, -1)

#### DETECT THE EDGES
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low = np.array([128,114,134])
    high = np.array([255,255,255])
    mask = cv2.inRange(hsv, low, high)

#### FIND THE CONTOURS
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key = lambda x:cv2.contourArea(x), reverse = True)
    for cnt in contours:
        (x,y,w,h) = cv2.boundingRect(cnt)
        #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(frame,center,radius,(0,255,0),2)
        break
    #a = cv2.line(frame, (x_medium, 0), (x_medium, 240), (0,255,0), 2)
    b = cv2.line(frame, (center1, 200), (center1, 270), (0,255,0), 2)
    #print(center-x_medium)

#### FIND THE DEGRESS OF THE OBJECT LOCATED FROM THE CENTRE OF THE CAMERA FRAME
    degrees = (int(x) - (center1 / 2)) * 0.061
    #print("degrees", degrees)

#### THRESHOLD VALUE
    center_degree = 10  

#### OBJECT IF LOCATED AT LEFT
    if center_degree > degrees+2:
        angleL = (center_degree - degrees)
        print("Angle in the left = ",angleL)
        timeL = ((angleL * 1.3) / 90)
        print("Time for the robot to turn to the required angle",timeL)
        left(timeL)
        print("The ball is in the left")

#### OBJECT IF LOCATED AT RIGHT
    if center_degree < degrees-2:
        angleR = center_degree - degrees
        print("Angle in the right = ",angleR)
        timeR = abs(((angleR * 1.3) / 90))
        print("Time for the robot to turn to the required angle",timeR)
        right(timeR)
        print("The ball is in the right")

#### SHOW THE OUTPUT
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    tf = 1
    
#### CHECK IF THE DISTANCE OF THE IDENTIFIED OBJECT NEAR OR FAR 
    if center_degree <= degrees+1 and center_degree >= degrees-1:
        distance_final = np.array([])
        for i in range(1,11):
            print("Distance: ",distance(), "cm")
            distance_final = np.append(distance_final, distance())
        m = distance_final
        mean = np.mean(m)
        print("mean = ", mean)

#### IF THE ROBOT IS NEAR THE OBJECT CAPTURE THE PHOTO AND SEND THE PHOTO TO THE REQUIRED MAIL ID (LOCALIZATION)
        if (mean < 12):
            init()
            print("THE OBJECT IS NEAR")
            cap.release()
            gameover()
            time.sleep(1)
            pwm_close(tf)

            # TIME AT WHICH THE IMAGE IS CAPTURED
            pic_time = datetime.now().strftime('%Y%m%d%H%M%S')
            # CAPTURE THE IMAGE
            command = 'raspistill -w 1280 -h 720 -vf -hf -o ' + pic_time + '.jpg'
            os.system(command)
            # SENDER'S MAIL ID AND PASSWORD
            smtpUser = 'juggernauteamkm@gmail.com'
            smtpPass = 'juggernaut@kmkm1212'
            # RECEIVERS MAIL ID
            toAdd = ['kas1455@terpmail.umd.edu','arvind1455@yahoo.in'] 
            fromAdd = smtpUser
            subject = 'Image recorded at ' + pic_time
            msg = MIMEMultipart()
            msg['Subject'] = subject
            msg['From'] = fromAdd
#msg['To'] = toAdd
            msg['To'] = ",".join(toAdd)
            msg.preamble = "IMAGE RECORDED AT " + pic_time
            body = MIMEText("Image recorded at " + pic_time + "and recorded by Megha Puthenparampil and Kamesh Arvind Sarangan")
            msg.attach(body)
            fp = open(pic_time +'.jpg','rb')
            img = MIMEImage(fp.read())
            fp.close()
            msg.attach(img)
            s = smtplib.SMTP('smtp.gmail.com', 587)
            s.ehlo()
            s.starttls()
            s.ehlo()
            s.login(smtpUser, smtpPass)
            s.sendmail(fromAdd, toAdd, msg.as_string())
            s.quit()
            print("Email Delivered!")
            time.sleep(1)
            reverse()
            time.sleep(1)
            pwm_open(tf)
            break

#### IF THE OBJECT IS AT A DISTANCE
        elif(distance() > 12):
            init()
            print("THE OBJECT IS FAR")
            forward()
            pwm_open(tf)
        
pwmservo.stop()
cap.release()        

