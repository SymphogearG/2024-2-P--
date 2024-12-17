
import cv2
import numpy as np
import YB_Pcb_Car
import time
import threading
import RPi.GPIO as GPIO
car = YB_Pcb_Car.YB_Pcb_Car()
def sbeep_sound():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(32, GPIO.OUT)
    p = GPIO.PWM(32, 440)
    p.start(50)
    time.sleep(0.3)
    p.stop()
    time.sleep(0.5)
    p.start(30)
    time.sleep(0.3)
    p.stop()
    time.sleep(0.5)
    p.start(30)
    time.sleep(0.3)
    p.stop()
    GPIO.cleanup()
    
def dbeep_sound():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(32, GPIO.OUT)
    p = GPIO.PWM(32, 440)
    p.start(50)
    time.sleep(0.3)
    p.stop()
    time.sleep(0.5)
    p.start(75)
    time.sleep(0.5)
    p.stop()
    time.sleep(0.5)
    p.start(95)
    time.sleep(0.6)
    p.stop()
    GPIO.cleanup()#여기 까지 안빼주세요



def decide_direction(histogram, frame, direction_threshold, car, up_threshold): #방향 결정함수
    length = len(histogram)
    left = int(np.sum(histogram[:length // 2]))
    right = int(np.sum(histogram[length // 2:]))
    center = int(np.sum(histogram[length // 3:2 * length // 3]))

    print("left:", left, "right:", right, "center:", center)
    if center < 5800000:
        section_width = frame.shape[1] // 20
        left_sum = 0
        right_sum = 0

        for i in range(20):
            mid_x = i * section_width + section_width // 2
            start_y = 0
            end_y = frame.shape[0] - 1
            section_line = frame[max(start_y, 35):min(end_y, frame.shape[0] - 35), max(mid_x - 1, 0):min(mid_x + 2, frame.shape[1])]

            black_pixels = np.argwhere(section_line == 0)

            if len(black_pixels) > 0:
                max_y = np.max(black_pixels[:, 0])
                print(f"Section {i}: Max Y-coordinate: {max_y}")
                if i < 10:
                    left_sum += max_y
                else:
                    right_sum += max_y

        print(f"Left sum: {left_sum}, Right sum: {right_sum}")

        if left_sum > right_sum:
            return "RIGHT"
        else:
            return "LEFT" #카메라에 잡히는 black_pixels을 이용하여 차량의 제자리 선회방향을 결정하는 함수

    if abs(right - left) > 800000:
        return "hRIGHT" if right > left else "hLEFT" #직진도로에 가까운 도로에서 벽을 만났을때 앞으로 나아가며 직진방향을 수정하도록 하는 함수

    return "UP" #직진함수

def control_car(direction, up_speed, down_speed):
    global current_direction  # Access the global direction variable
    current_direction = direction  # Update the direction
    print(f"Current Direction: {current_direction}")  # Print the direction to Shell

    if direction == "UP":
        car.Car_Run(50, 50)
    elif direction == "LEFT":
        car.Car_Left(57, 57)
    elif direction == "RIGHT":
        car.Car_Right(57, 57)
    elif direction == "hLEFT":
        car.Car_Run(25, 66)
    elif direction == "hRIGHT":
        car.Car_Run(66, 25)
    elif direction == "STOP":
        car.Car_Stop()

    while True:#이 부분 빼주세요
        stop_cascade = cv2.CascadeClassifier('stop_cascade.xml')
        dan_cascade = cv2.CascadeClassifier('dan_cascade.xml')
        o_cascade = cv2.CascadeClassifier('o_cascade.xml')
        down_cascade = cv2.CascadeClassifier('down_cascade.xml')
        dan_signs = dan_cascade.detectMultiScale(gray_origin, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        stop_signs = dan_cascade.detectMultiScale(gray_origin, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        o_signs = o_cascade.detectMultiScale(gray_origin, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        down_signs = down_cascade.detectMultiScale(gray_origin, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)) #여기까지 뺴주세요

        
        
        if len(o_signs) > 0: #화면에서 o_signs을 감지
            print("O detected! Aligning car.")
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Camera read failed.")
                    break

                gray_origin = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                o_signs = o_cascade.detectMultiScale(gray_origin, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)) #화면에서 o_signs을 감지

                if len(o_signs) == 0:
                    print("O no longer detected. Stopping car.")
                    car.Car_Stop()
                  

                for (x, y, w, h) in o_signs:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # o_signs에 초록색 사각형을 그림
                    center_x = x + w // 2
                    frame_center_x = gray_origin.shape[1] // 2 

                    if abs(center_x - frame_center_x) > 10: 
                        if center_x < frame_center_x:
                            print("Turning left...")
                            car.Car_Left(50, 50)  
                        else:
                            print("Turning right...")
                            car.Car_Right(50, 50)  # 인시한 o_signs이 카메라의 정중앙에 오도록 차량을 선회
                    else:
                        print("Aligned. Moving forward...")
                        car.Car_Run(60, 60)
                        time.sleep(1)
                        car.Car_Stop()
                        time.sleep(20) #차량을 전진시켜 주차
                        break
                    
        elif len(stop_signs) > 0 and stop != 1: #stop_signs을 인식
            stop = 1
            print("Stop sign detected! Stopping the car.")
            for (x, y, w, h) in stop_signs:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  #stop_signs에 초록색 사각형을 그림
            car.Car_Stop()
            sbeep_sound() #비프음 출력
           

        elif len(dan_signs) > 0 and dan != 1: #dan_signs을 인식
            dan = 1
            print("Danger sign detected! Stopping the car.")
            for (x, y, w, h) in dan_signs:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  #dan_signs에 초록색 사각형을 그림
            car.Car_Stop()
            dbeep_sound() #비프음 출력
            
            

    