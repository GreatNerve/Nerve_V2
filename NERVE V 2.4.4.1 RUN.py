# NERVE V  2.4.4.1 RUN is Deigned by Dheeraj Sharma at 2 October 2019 04:39(IST) with audio.
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
from pygame import mixer
from Bluetin_Echo import Echo
from mfrc522 import SimpleMFRC522
import cv2
import time
import os
import socket
import sys
from _thread import *

WIDTH = 800
HEIGHT = 464

camera = PiCamera()
camera.resolution = (WIDTH, HEIGHT)
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

mixer.init(0)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 16
GPIO_ECHO = 20
GPIO_SERVO = 18
GPIO_ALCHOLE = 21
GPIO_PIR = 26
GPIO_SHUTDOWN=4
GPIO_SPEED = 13
GPIO_IN1 = 17
GPIO_IN2 = 19
GPIO_IN3 = 27
GPIO_IN4 = 24
speed_of_sound = 343
GPIO.setup(GPIO_PIR,GPIO.IN)
GPIO.setup(GPIO_ALCHOLE,GPIO.IN)
GPIO.setup(GPIO_SHUTDOWN,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_SPEED, GPIO.OUT)
GPIO.setup(GPIO_SERVO, GPIO.OUT)
GPIO.setup(GPIO_IN1,GPIO.OUT)
GPIO.setup(GPIO_IN2,GPIO.OUT)
GPIO.setup(GPIO_IN3,GPIO.OUT)
GPIO.setup(GPIO_IN4,GPIO.OUT)
reader = SimpleMFRC522()
sonar = Echo(GPIO_TRIGGER, GPIO_ECHO, speed_of_sound)

font = cv2.FONT_HERSHEY_SIMPLEX
host = ''
port = 5555
UP_Key=82
Down_Key=84
Left_Key=81
Right_Key=83
Q_Key=113
R_Key=114
C_Key= 99
E_Key=101
T_Key=116
A_Key=97
M_Key = 109
Enter_Key=13
M_Key = 109
P_Key = 112
K_Key = 107
Distance_Limit=30
ID_Card= 1032293622389

speed = GPIO.PWM(GPIO_SPEED, 100) #
speed.start(0)
servo = GPIO.PWM(GPIO_SERVO, 50) # GPIO 17 for PWM with 50Hz
servo.start(6.25) # Initialization

def Dheeraj_Sharma():
    Mode =0;File_Status = 0
    global conn,addr,Telnet,speed_value
    start =0.00;stop =0.00;i=0;time_mode=0;run=0;PIR_Mode=0;time_start=0.00;time_stop=0.00;Telnet=0
    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/RFID Welcome.wav')
    sound.play()
    GPIO.output(GPIO_IN1,GPIO.LOW)
    GPIO.output(GPIO_IN2,GPIO.LOW)
    GPIO.output(GPIO_IN3,GPIO.LOW)
    GPIO.output(GPIO_IN4,GPIO.LOW)
    print('NERVE V 2.4.4.1 RUN')
    print('Please Scan RFID Card')
    while True:
        ID, text = reader.read()
        if ID ==ID_Card:
            print('ID is Match')
            print('Welcome Dheeraj Sharma')
            break
        else :
            sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Permission Decline.wav')
            sound.play()
            print('Permission Decline')
            time.sleep(1)
    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Route Enter.wav')
    sound.play()
    Data=input("Enter Route Name : ")
    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Speed Enter.wav')
    sound.play()
    speed_value=int(input("Enter Speed from 0% to 100% : "))
    if(speed_value<0):
        speed_value=1
    elif(speed_value>100):
        speed_value=100
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind((host, port))
    except socket.error as e:
        print(str(e))
    String = 'Welcome'
    s.listen(10)
    try:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            servo.ChangeDutyCycle(6.25)
            PIR = GPIO.input(GPIO_PIR)
            if(PIR==0 or PIR_Mode==1):
                ALCHOLE = GPIO.input(GPIO_ALCHOLE)
                ALCHOLE =1
                char =  cv2.waitKey(1) & 0xFF
                #print(char)
                if (char == Q_Key):
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    break
                elif (char ==R_Key) :
                    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Remote Welcome.wav')
                    sound.play()
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    print('Remote Driving Mode is Activate Successfully')
                    String ='Remote Driving Mode is Activate Successfully'
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    file = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+".txt","w")
                    delay = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+"_Time.txt","w")
                    start =0.00;stop =0.00;i=0;File_Status = 1;time_mode =0
                    if(ALCHOLE==1):
                        Mode=1
                    else :
                        Mode=0
                elif (char == E_Key) :
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    String ='Exit'
                    print('Exit')
                    File_Status = 0;Mode=0;time_mode=0
                elif (char == T_Key) :
                    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Code Welcome.wav')
                    sound.play()
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    print('Code Driving Mode is Activate Successfully')
                    String ='Code Driving Mode is Activate Successfully'
                    file = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+".txt","r")
                    delay = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+"_Time.txt","r")
                    File_Status = 1;time_mode =0;Mode=2
                elif (char == A_Key):
                    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Auto Welcome.wav')
                    sound.play()
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    print('Auto Driving Mode is Activate Successfully')
                    String ='Auto Driving Mode is Activate Successfully'
                    File_Status = 0;time_mode =0;Mode=3
                elif (char == C_Key):
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    cv2.destroyAllWindows()
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    Mode=0;File_Status = 0;time_mode=0
                    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Route Enter.wav')
                    sound.play()
                    Data=input("Enter Route Name : ")
                    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Speed Enter.wav')
                    sound.play()
                    speed_value=int(input("Enter Speed from 0% to 100% : "))
                    if(speed_value<0):
                        speed_value=1
                    elif(speed_value>100):
                        speed_value=100
                elif (char==M_Key):
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    if(run==0):
                        sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Motor Lock Disable.wav')
                        sound.play()
                        speed.ChangeDutyCycle(speed_value)
                        run=1
                    else:
                        sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Motor lock enable.wav')
                        sound.play()
                        speed.ChangeDutyCycle(0)
                        run=0
                    print('Run Activate ',run)
                    String = 'Run Activate '+str(run)
                elif (char==P_Key):
                    if(PIR_Mode==0):
                        PIR_Mode=1
                    else:
                        PIR_Mode=0
                    print('PIR Deactivate ',PIR_Mode)
                    String = 'PIR Deactivate '+str(PIR_Mode)
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                elif (char==K_Key):
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                    if(Telnet==0):
                        print('Telnet Port is Activate 1')
                        String = 'Telnet Port is Activate 1'
                        conn, addr = s.accept()
                        Telnet=1
                    else:
                        print('Telnet Port is Activate 0')
                        String = 'Telnet Port is Activate 0'
                        Telnet=0

                if(distance_cm()>Distance_Limit or Mode ==3):
                    if (char == UP_Key) and (Mode==1):
                        if i==1 :
                            stop = time.time()
                            delay.write(str(((stop-start)*speed_value))+'\n')
                        start = time.time()
                        file.write('f\n')
                        start_new_thread(threaded_client,(0,'Forward\n'))
                        String ='Forward'
                        print('Forward')
                        GPIO.output(GPIO_IN1,GPIO.HIGH)
                        GPIO.output(GPIO_IN2,GPIO.LOW)
                        GPIO.output(GPIO_IN3,GPIO.HIGH)
                        GPIO.output(GPIO_IN4,GPIO.LOW)
                        i=1
                    elif (char == Down_Key )and(Mode==1):
                        if i==1 :
                            stop = time.time()
                            delay.write(str(((stop-start)*speed_value))+'\n')
                        start = time.time()
                        file.write('b\n')
                        start_new_thread(threaded_client,(0,'Backward\n'))
                        String ='Backward'
                        print('Backward')
                        GPIO.output(GPIO_IN1,GPIO.LOW)
                        GPIO.output(GPIO_IN2,GPIO.HIGH)
                        GPIO.output(GPIO_IN3,GPIO.LOW)
                        GPIO.output(GPIO_IN4,GPIO.HIGH)
                        i=1
                    elif (char == Right_Key)and(Mode==1):
                        if i==1 :
                            stop = time.time()
                            delay.write(str(((stop-start)*speed_value))+'\n')
                        start_new_thread(threaded_client,(0,'Right\n'))
                        start = time.time()
                        file.write('r\n')
                        String ='Right'
                        print('Right')
                        GPIO.output(GPIO_IN1,GPIO.LOW)
                        GPIO.output(GPIO_IN2,GPIO.LOW)
                        GPIO.output(GPIO_IN3,GPIO.HIGH)
                        GPIO.output(GPIO_IN4,GPIO.LOW)
                        i=1
                    elif (char == Left_Key)and(Mode==1):
                        if i ==1:
                            stop = time.time()
                            delay.write(str(((stop-start)*speed_value))+'\n')
                        start = time.time()
                        file.write('l\n')
                        start_new_thread(threaded_client,(0,'Left\n'))
                        String ='Left'
                        print('Left')
                        GPIO.output(GPIO_IN1,GPIO.HIGH)
                        GPIO.output(GPIO_IN2,GPIO.LOW)
                        GPIO.output(GPIO_IN3,GPIO.LOW)
                        GPIO.output(GPIO_IN4,GPIO.LOW)
                        i=1
                    elif (char == Enter_Key)and(Mode==1):
                        GPIO.output(GPIO_IN1,GPIO.LOW)
                        GPIO.output(GPIO_IN2,GPIO.LOW)
                        GPIO.output(GPIO_IN3,GPIO.LOW)
                        GPIO.output(GPIO_IN4,GPIO.LOW)
                        if i ==1:
                            stop = time.time()
                            delay.write(str(((stop-start)*speed_value))+'\n')
                        start = time.time()
                        file.write('s\n')
                        start_new_thread(threaded_client,(0,'Stop\n'))
                        String ='Stop'
                        print('Stop')
                        i=1;
                    if(Mode==2):
                        data = file.readline()
                        if data == 'f\n':
                            start_new_thread(threaded_client,(0,'Forward\n'))
                            print('Forward')
                            String ='Forward'
                            GPIO.output(GPIO_IN1,GPIO.HIGH)
                            GPIO.output(GPIO_IN2,GPIO.LOW)
                            GPIO.output(GPIO_IN3,GPIO.HIGH)
                            GPIO.output(GPIO_IN4,GPIO.LOW)
                        elif data =='b\n':
                            print('Backward')
                            String ='Backward'
                            start_new_thread(threaded_client,(0,'Backward\n'))
                            GPIO.output(GPIO_IN1,GPIO.LOW)
                            GPIO.output(GPIO_IN2,GPIO.HIGH)
                            GPIO.output(GPIO_IN3,GPIO.LOW)
                            GPIO.output(GPIO_IN4,GPIO.HIGH)
                        elif data =='l\n':
                            start_new_thread(threaded_client,(0,'Left\n'))
                            print('Left')
                            String ='Left'
                            GPIO.output(GPIO_IN1,GPIO.HIGH)
                            GPIO.output(GPIO_IN2,GPIO.LOW)
                            GPIO.output(GPIO_IN3,GPIO.LOW)
                            GPIO.output(GPIO_IN4,GPIO.LOW)
                        elif data =='r\n':
                            start_new_thread(threaded_client,(0,'Right\n'))
                            print('Right')
                            String ='Right'
                            GPIO.output(GPIO_IN1,GPIO.LOW)
                            GPIO.output(GPIO_IN2,GPIO.LOW)
                            GPIO.output(GPIO_IN3,GPIO.HIGH)
                            GPIO.output(GPIO_IN4,GPIO.LOW)
                        elif data =='s\n':
                            print('Stop')
                            String ='Stop'
                            start_new_thread(threaded_client,(0,'Stop\n'))
                            GPIO.output(GPIO_IN1,GPIO.LOW)
                            GPIO.output(GPIO_IN2,GPIO.LOW)
                            GPIO.output(GPIO_IN3,GPIO.LOW)
                            GPIO.output(GPIO_IN4,GPIO.LOW)
                        elif data =='' :
                            print('File is finised')
                            String ='File is finised'
                            start_new_thread(threaded_client,(0,'Stop\n'))
                            GPIO.output(GPIO_IN1,GPIO.LOW)
                            GPIO.output(GPIO_IN2,GPIO.LOW)
                            GPIO.output(GPIO_IN3,GPIO.LOW)
                            GPIO.output(GPIO_IN4,GPIO.LOW)
                            Mode=0
                        start = time.time()
                        s=delay.readline()
                        s=s.rstrip('\n')
                        time_mode=1
                    if(time_mode==1):
                        if( s !='' ):
                            s=float(s)
                            stop = time.time()
                            if(s<=((stop-start)*speed_value)):
                                Mode=2
                            else :
                                Mode=0
                    #ALCHOLE=0
                    if(ALCHOLE==0 and Mode==1):
                        Mode=0
                        print('Alchole Detect')
                        String+=' , Alchole Detect'
                    if(run==1):
                        speed.ChangeDutyCycle(speed_value)
                    else:
                        speed.ChangeDutyCycle(0)
                else :
                    sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Object Detected.wav')
                    sound.play()
                    print('Object Detected')
                    String='Object Detected'
                    start_new_thread(threaded_client,(0,'Backward\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.HIGH)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.HIGH)
                    time.sleep(1)
                    start_new_thread(threaded_client,(0,'Stop\n'))
                    GPIO.output(GPIO_IN1,GPIO.LOW)
                    GPIO.output(GPIO_IN2,GPIO.LOW)
                    GPIO.output(GPIO_IN3,GPIO.LOW)
                    GPIO.output(GPIO_IN4,GPIO.LOW)
                if Mode ==3:
                    String = Auto_driveing_mode()

            else:
                sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Human Detected.wav')
                sound.play()
                print('Humen Detect')
                String= 'Humen Detect'
                start_new_thread(threaded_client,(0,'Stop\n'))
                GPIO.output(GPIO_IN1,GPIO.LOW)
                GPIO.output(GPIO_IN2,GPIO.LOW)
                GPIO.output(GPIO_IN3,GPIO.LOW)
                GPIO.output(GPIO_IN4,GPIO.LOW)
            #String = str(distance_cm())
            if(GPIO.input(GPIO_SHUTDOWN)==1):
                start_time= time.time()
            else:
                Mode=0;time_mode=0
                start_new_thread(threaded_client,(0,'Stop\n'))
                print('Break')
                string = 'Break'
                GPIO.output(GPIO_IN1,GPIO.LOW)
                GPIO.output(GPIO_IN2,GPIO.LOW)
                GPIO.output(GPIO_IN3,GPIO.LOW)
                GPIO.output(GPIO_IN4,GPIO.LOW)
                stop_time= time.time()
                if((stop_time-start_time)>10):
                    start_new_thread(threaded_client,(0,'Shutdown\n'))
                    print('Shutdown')
                    String='Shutdown'
                    if(File_Status == 1):
                        file.close()
                        delay.close()
                    time.sleep(2)
                    os.system('sudo shutdown now')
            cv2.putText(image,String,(10,26),font,0.7,(12,243,26),2,cv2.LINE_AA)
            cv2.imshow("NERVE Camera Vision", image)
            rawCapture.truncate(0)

    finally:
        #conn.close()
        sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/End.wav')
        sound.play()
        servo.stop()
        sonar.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        print('Program close by Dheeraj Sharma')
        time.sleep(3.5)
def Auto_driveing_mode():
    servo.ChangeDutyCycle(6.25)
    distance = distance_cm()
   # print(distance1)
    if distance <=Distance_Limit:
        start_new_thread(threaded_client,(0,'Backward\n'))
        print('Backward')
        string = 'Backward'
        GPIO.output(GPIO_IN1,GPIO.LOW)
        GPIO.output(GPIO_IN2,GPIO.HIGH)
        GPIO.output(GPIO_IN3,GPIO.LOW)
        GPIO.output(GPIO_IN4,GPIO.HIGH)
        time.sleep(0.5)
        start_new_thread(threaded_client,(0,'Stop\n'))
        print('Stop')
        string = 'Stop'
        GPIO.output(GPIO_IN1,GPIO.LOW)
        GPIO.output(GPIO_IN2,GPIO.LOW)
        GPIO.output(GPIO_IN3,GPIO.LOW)
        GPIO.output(GPIO_IN4,GPIO.LOW)
        servo.ChangeDutyCycle(12.0)
        time.sleep(0.5)
        left_distance = distance_cm()
        time.sleep(0.5)
        servo.ChangeDutyCycle(3.0)
        time.sleep(0.5)
        right_distance = distance_cm()
        time.sleep(0.5)
        if right_distance > left_distance :
            servo.ChangeDutyCycle(6.25)
            start_new_thread(threaded_client,(0,'Right\n'))
            print('Right')
            string = 'Right'
            GPIO.output(GPIO_IN1,GPIO.LOW)
            GPIO.output(GPIO_IN2,GPIO.LOW)
            GPIO.output(GPIO_IN3,GPIO.HIGH)
            GPIO.output(GPIO_IN4,GPIO.LOW)
            time.sleep(1)
        elif right_distance < left_distance :
            servo.ChangeDutyCycle(6.25)
            start_new_thread(threaded_client,(0,'left\n'))
            print('left')
            string = 'left'
            GPIO.output(GPIO_IN1,GPIO.HIGH)
            GPIO.output(GPIO_IN2,GPIO.LOW)
            GPIO.output(GPIO_IN3,GPIO.LOW)
            GPIO.output(GPIO_IN4,GPIO.LOW)
            time.sleep(1)
        return string
    else:
        start_new_thread(threaded_client,(0,'Forward\n'))
        print('Forward')
        string = 'Forward'
        GPIO.output(GPIO_IN1,GPIO.HIGH)
        GPIO.output(GPIO_IN2,GPIO.LOW)
        GPIO.output(GPIO_IN3,GPIO.HIGH)
        GPIO.output(GPIO_IN4,GPIO.LOW)
        return string
def distance_cm():
    time.sleep(0.2)
    distance = sonar.read('cm')
    if distance > 400 or distance ==0:
        distance =400
    return int(distance)
def threaded_client(c,reply):
    if Telnet :
        conn.sendall(str.encode(reply))


if __name__ == '__main__':
    Dheeraj_Sharma()