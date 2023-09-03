# NERVE V2.4 TEST is Deigned by Dheeraj Sharma at 28 July 2019 3:44(IST) without audio.
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
from pygame import mixer
from Bluetin_Echo import Echo
import cv2
import time


WIDTH = 800
HEIGHT = 464

camera = PiCamera()
camera.vflip = True
camera.hflip = True
camera.resolution = (WIDTH, HEIGHT)
rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

#mixer.init(0)


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 16
GPIO_ECHO = 20
GPIO_SERVO = 18
speed_of_sound = 343
sonar = Echo(GPIO_TRIGGER, GPIO_ECHO, speed_of_sound)

font = cv2.FONT_HERSHEY_SIMPLEX
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
Enter_Key=13

GPIO.setup(GPIO_SERVO, GPIO.OUT)
servo = GPIO.PWM(GPIO_SERVO, 50) # GPIO 17 for PWM with 50Hz
servo.start(6.25) # Initialization

def Dheeraj_Sharma():
    Mode =0
    File_Status = 0
    start =0.00;stop =0.00;i=0;time_mode=0
    Data=input("Enter Route Name : ")
    String = 'Welcome'
    try: 
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            
            char =  cv2.waitKey(1) & 0xFF
            servo.ChangeDutyCycle(6.25)
            #print(char)
            if (char == Q_Key):
                if(File_Status == 1):
                    file.close()
                    delay.close()
                break
            elif (char ==R_Key) :
                print('Remote Driving Mode is Activate Successfully')
                String ='Remote Driving Mode is Activate Successfully'
                if(File_Status == 1):
                    file.close()
                    delay.close()
                file = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+".txt","w")
                delay = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+"_Time.txt","w")
                start =0.00;stop =0.00;i=0
                File_Status = 1
                Mode=1
                time_mode =0
            elif (char == E_Key) :
                if(File_Status == 1):
                    file.close()
                    delay.close()
                String ='Exit'
                print('Exit')
                File_Status = 0
                Mode=0
                time_mode=0
            elif (char == T_Key) :
                if(File_Status == 1):
                    file.close()
                    delay.close()
                print('Code Driving Mode is Activate Successfully')
                String ='Code Driving Mode is Activate Successfully'
                file = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+".txt","r")
                delay = open(r"/home/pi/Desktop/NERVE V 2 TEST/Data/"+Data+"_Time.txt","r")
                File_Status = 1
                time_mode =0
                Mode=2
            elif (char == A_Key):
                if(File_Status == 1):
                    file.close()
                    delay.close()
                print('Auto Driving Mode is Activate Successfully')
                String ='Auto Driving Mode is Activate Successfully'
                File_Status = 0
                time_mode =0
                Mode=3
            elif (char == C_Key): 
                cv2.destroyAllWindows()
                if(File_Status == 1):
                    file.close()
                    delay.close()
                Mode=0
                File_Status = 0
                Data=input("Enter Route Name : ")
            elif (char == UP_Key) and (Mode==1):
                if i==1 :
                    stop = time.time()
                    delay.write(str(stop-start)+'\n')
                start = time.time()
                file.write('f\n')
                String ='Forward'
                print('Forward')
                i=1
            elif (char == Down_Key )and(Mode==1):
                if i==1 :
                    stop = time.time()
                    delay.write(str(stop-start)+'\n')
                start = time.time()
                file.write('b\n')
                String ='Backward'
                print('Backward')
                i=1
            elif (char == Right_Key)and(Mode==1):
                if i==1 :
                    stop = time.time()
                    delay.write(str(stop-start)+'\n')
                start = time.time()
                file.write('r\n')
                String ='Right'
                print('Right')
            elif (char == Left_Key)and(Mode==1):
                if i ==1:
                    stop = time.time()
                    delay.write(str(stop-start)+'\n')
                start = time.time()
                file.write('l\n')
                String ='Left'
                print('Left')
                i=1
            elif (char == Enter_Key)and(Mode==1):
                if i ==1:
                    stop = time.time()
                    delay.write(str(stop-start)+'\n')
                start = time.time()
                file.write('s\n')
                String ='Stop'
                print('Stop')
                i=1;
            if(Mode==2):
                data = file.readline()
                if data == 'f\n':
                    print('Forward')
                    String ='Forward'
                elif data =='b\n':
                    print('Backward')
                    String ='Backward'
                elif data =='l\n':
                    print('Left')
                    String ='Left'
                elif data =='r\n':
                    print('Right')
                    String ='Right'
                elif data =='s\n':
                    print('Stop')
                    String ='Stop'
                elif data =='' :
                    print('File is finised')
                    String ='File is finised'
                    Mode=0
                start = time.time()
                s=delay.readline()
                s=s.rstrip('\n')
                time_mode=1
            elif Mode ==3:
                String = Auto_driveing_mode()
            if(time_mode==1):
                if s !='' :
                    s=float(s)
                    stop = time.time()
                    if(s<=(stop-start)):
                        Mode=2
                    else :
                        Mode=0
            #String = str(distance_cm())
            cv2.putText(image,String,(10,26),font,0.7,(12,243,26),2,cv2.LINE_AA)
            cv2.imshow("Frame", image)
            rawCapture.truncate(0)
            
    finally:
        servo.stop()
        sonar.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
#        sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/End.wav')
#        sound.play()
        print('Program close by Dheeraj Sharma')
def Auto_driveing_mode():
    distance = distance_cm()
   # print(distance1)
    if distance <=10:
        print('backword')
        string = 'backword'
        servo.start(12.5)
        time.sleep(1.5)
        left_distance = distance_cm()
        #time.sleep(1.5)
        servo.start(2.5)
        time.sleep(1.5)
        right_distance = distance_cm()
        #time.sleep(1.5)
        if right_distance > left_distance :
            print('right')
        elif right_distance < left_distance :
            print('left')
def distance_cm():
    distance = sonar.read('cm')
    if distance > 400 or distance ==0:
        distance =400
    return int(distance)



if __name__ == '__main__':
    Dheeraj_Sharma()
