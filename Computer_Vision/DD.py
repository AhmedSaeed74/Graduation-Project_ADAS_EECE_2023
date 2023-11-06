from scipy.spatial import distance
from imutils import face_utils
import imutils
import dlib
import cv2
import numpy as np
from math import hypot
import os
import can
import time
from random import randrange
import serial
from time import sleep
import firebase_admin
from firebase_admin import credentials, messaging
from firebase_admin import firestore

ser = serial.Serial ("/dev/ttyS0", 115200)    #Open port with baud rate

normal_status = 0
warning_status = 1
full_brake_status = 2

drowsiness_old_status = normal_status
distraction_old_status = normal_status

zeros_counter = 0
distraction_counter = 0

end_simulation_in_30_seconds = 0

os.system("sudo /sbin/ip link set can0 down")
os.system("sudo ip link set can0 up type can bitrate 500000")

def emergenecy_call():
    received_data = ser.read()              #read serial port
    sleep(0.03)
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    frame = received_data.decode()

    if frame.startswith("*") and "#" in frame:
        # Extracting x
        carID_start = frame.find("*") + 1
        carID_end = frame.find(";")
        carID = int(frame[carID_start:carID_end])

        # Extracting y
        x_start = carID_end + 1
        x_end = frame.find("/")
        x = float(frame[x_start:x_end])

        # Extracting z
        y_start = x_end + 1
        y_end = frame.find("#")
        y = float(frame[y_start:y_end])

        print("")
        print(f"Car ID = {carID}")
        print(f"X Coordinate = {x}")
        print(f"Y Coordinate = {y}")
                # Initialize Firebase Admin SDK
        cred = credentials.Certificate('serviceAccountKey.json')
        firebase_admin.initialize_app(cred)

        db = firestore.client()
        doc_ref = db.collection('emergency').document()
        uid = doc_ref.id
        # Push data to a specific collection
        data = {'x': f"{x}",
                'y': f"{y}",
                'carId': f"{carID}",

                'uid': f"{uid}",
                'status': False,
                'user': {'uid': "NULL"},
                'hospital': {'uid': "NULL"},

                }

        doc_ref.set(data)

        message = messaging.Message(
            notification=messaging.Notification(
                title='New Accident',
                body=f"Driver with CarID '{carID}' make Accident , Try to Help",
            ),
            topic='all'
        )

        # Send the message
        response = messaging.send(message)
        print('Notification sent:', response)

def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    EAR = (A + B) / (2.0 * C)
    return EAR
    
def midpoint(p1 ,p2):
    return int((p1.x + p2.x)/2), int((p1.y + p2.y)/2)

font = cv2.FONT_HERSHEY_PLAIN

# def get_blinking_ratio(eye_points, facial_landmarks):
    # left_point = (facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y)
    # right_point = (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y)
    # center_top = midpoint(facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2]))
    # center_bottom = midpoint(facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4]))

    # #hor_line = cv2.line(frame, left_point, right_point, (0, 255, 0), 2)
    # #ver_line = cv2.line(frame, center_top, center_bottom, (0, 255, 0), 2)

    # hor_line_lenght = hypot((left_point[0] - right_point[0]), (left_point[1] - right_point[1]))
    # ver_line_lenght = hypot((center_top[0] - center_bottom[0]), (center_top[1] - center_bottom[1]))

    # ratio = hor_line_lenght / ver_line_lenght
    # return ratio

def get_gaze_ratio(eye_points, facial_landmarks):
    left_eye_region = np.array([(facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y),
                                (facial_landmarks.part(eye_points[1]).x, facial_landmarks.part(eye_points[1]).y),
                                (facial_landmarks.part(eye_points[2]).x, facial_landmarks.part(eye_points[2]).y),
                                (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y),
                                (facial_landmarks.part(eye_points[4]).x, facial_landmarks.part(eye_points[4]).y),
                                (facial_landmarks.part(eye_points[5]).x, facial_landmarks.part(eye_points[5]).y)], np.int32)
    # cv2.polylines(frame, [left_eye_region], True, (0, 0, 255), 2)

    height, width, _ = frame.shape
    mask = np.zeros((height, width), np.uint8)
    cv2.polylines(mask, [left_eye_region], True, 255, 2)
    cv2.fillPoly(mask, [left_eye_region], 255)
    eye = cv2.bitwise_and(gray, gray, mask=mask)

    min_x = np.min(left_eye_region[:, 0])
    max_x = np.max(left_eye_region[:, 0])
    min_y = np.min(left_eye_region[:, 1])
    max_y = np.max(left_eye_region[:, 1])

    gray_eye = eye[min_y: max_y, min_x: max_x]
    _, threshold_eye = cv2.threshold(gray_eye, 70, 255, cv2.THRESH_BINARY)
    height, width = threshold_eye.shape
    left_side_threshold = threshold_eye[0: height, 0: int(width / 2)]
    left_side_white = cv2.countNonZero(left_side_threshold)

    right_side_threshold = threshold_eye[0: height, int(width / 2): width]
    right_side_white = cv2.countNonZero(right_side_threshold)

    if left_side_white == 0:
        gaze_ratio = 1
    elif right_side_white == 0:
        gaze_ratio = 5
    else:
        gaze_ratio = left_side_white / right_side_white
    return gaze_ratio    

thresh = 0.22
frame_check = 20
num_of_frames = 0
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

#loading pre trained face data from opencv
trained_data = cv2.CascadeClassifier('frontal-face-data.xml')

(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
cap = cv2.VideoCapture(0)
start_time = time.time()

flag = 0

while True:
    ret, frame = cap.read()
    new_frame = np.zeros((500, 500, 3), np.uint8)
    
    elapsed_time = time.time() - start_time
    fps = num_of_frames / elapsed_time
    print("Elapsed Time: {: .2f}".format(elapsed_time))
    print("FPS: {: .2f}".format(fps))
    
    frame = imutils.resize(frame, width=400)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    faces = detect(gray)
    for face in faces:
        #x, y = face.left(), face.top()
        #x1, y1 = face.right(), face.bottom()
        #cv2.rectangle(frame, (x, y), (x1, y1), (0, 255, 0), 2)

        landmarks = predict(gray, face)

        # Detect blinking
        # left_eye_ratio = get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks)
        # right_eye_ratio = get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks)
        # blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2

        # Gaze detection
    gaze_ratio_left_eye = get_gaze_ratio([36, 37, 38, 39, 40, 41], landmarks)
    gaze_ratio_right_eye = get_gaze_ratio([42, 43, 44, 45, 46, 47], landmarks)
    gaze_ratio = (gaze_ratio_right_eye + gaze_ratio_left_eye) / 2



    # if blinking_ratio > 5.7:
        # distraction_counter +=1
        # cv2.putText(frame, "BLINKING", (30, 80), font, 2, (0, 173, 255), 3)
    if gaze_ratio <= 0.8:
        distraction_counter +=1
        cv2.putText(frame, "RIGHT", (30, 80), font, 2, (0, 173, 255), 3)
        new_frame[:] = (0, 0, 255)
    elif 0.8 < gaze_ratio < 2:
        distraction_counter = 0
        cv2.putText(frame, "CENTER", (30, 80), font, 2, (0, 173, 255), 3)
    else:
        distraction_counter +=1
        new_frame[:] = (255, 0, 0)
        cv2.putText(frame, "LEFT", (30, 80), font, 2, (0, 173, 255), 3)
        
    # detect faces
    face_cordinates = trained_data.detectMultiScale(gray)
    #x and y is the upper left corner coordinate and w and h are the corresponding width and height of the rectangle
    for (x,y,w,h) in face_cordinates:
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        
    subjects = detect(gray, 0)
    
    for face in subjects:
        shape = predict
    
    for subject in subjects:
        shape = predict(gray, subject)
        shape = face_utils.shape_to_np(shape)
        leftEye = shape[lStart:lEnd]
        rightEye = shape[rStart:rEnd]
        leftEAR = eye_aspect_ratio(leftEye)
        rightEAR = eye_aspect_ratio(rightEye)
        EAR = (leftEAR + rightEAR) / 2.0
        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        #cv2.drawContours(frame, [leftEyeHull], -1, (255, 255, 255), 1)
        #cv2.drawContours(frame, [rightEyeHull], -1, (255, 255, 255), 1)
        #for(x , y) in shape:
            #cv2.circle(frame, (x , y), 2, (255, 255, 255), -1)
                
        if EAR < thresh:
            flag += 1
            EAR = 0
            zeros_counter +=1
            if flag >= 20:
                cv2.putText(frame, "***********ALERT!***********", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (46, 33, 235), 2)
                #cv2.putText(frame, "***********ALERT!***********", (10, 150),
                    #cv2.FONT_HERSHEY_SIMPLEX, 0.7, (46, 33, 235), 2)
                    
            elif flag >= 10:
                cv2.putText(frame, "**********WARNING!********", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (46, 33, 235), 2)
                #cv2.putText(frame, "**********WARNING!********", (10, 150),
                    #cv2.FONT_HERSHEY_SIMPLEX, 0.7, (46, 33, 235), 2)
        else:
            flag = 0
            EAR = 1
            zeros_counter = 0 
        
        # if zeros_counter >= 20 and distraction_counter >= 20:
            # if drowsiness_old_status != full_brake_status or distraction_old_status != full_brake_status:
                # os.system("sudo /sbin/ip link set can0 down")
                # os.system("sudo ip link set can0 up type can bitrate 500000")
                # os.system("cansend can0 123#0{}0{}".format(full_brake_status,full_brake_status))
                # drowsiness_old_status = full_brake_status
                # distraction_old_status = full_brake_status
                
        if zeros_counter >= 20:        
            if drowsiness_old_status != full_brake_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(full_brake_status,distraction_old_status))
                drowsiness_old_status = full_brake_status
                
        # elif distraction_counter >= 20:             
            # if distraction_old_status != full_brake_status:
                # os.system("sudo /sbin/ip link set can0 down")
                # os.system("sudo ip link set can0 up type can bitrate 500000")
                # os.system("cansend can0 123#0{}0{}".format(drowsiness_old_status,full_brake_status))
                # distraction_old_status = full_brake_status    
                
        elif zeros_counter >= 10 and distraction_counter >= 10:
            if drowsiness_old_status != warning_status or distraction_old_status != warning_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(warning_status,warning_status))
                drowsiness_old_status = warning_status
                distraction_old_status = warning_status
               
        elif zeros_counter >= 10:        
            if drowsiness_old_status != warning_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(warning_status, distraction_old_status))
                drowsiness_old_status = warning_status
                
        elif distraction_counter >= 10:              
            if distraction_old_status != warning_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(drowsiness_old_status, warning_status))
                distraction_old_status = warning_status  
                
        elif zeros_counter > 0 and distraction_counter > 0:
            if drowsiness_old_status != normal_status or distraction_old_status != normal_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(normal_status,normal_status))
                drowsiness_old_status = normal_status
                distraction_old_status = normal_status
                
        elif zeros_counter > 0:            
            if drowsiness_old_status != normal_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(normal_status,distraction_old_status))
                drowsiness_old_status = normal_status
                    
        elif distraction_counter > 0:            
            if distraction_old_status != normal_status:
                os.system("sudo /sbin/ip link set can0 down")
                os.system("sudo ip link set can0 up type can bitrate 500000")
                os.system("cansend can0 123#0{}0{}".format(drowsiness_old_status,normal_status))
                distraction_old_status = normal_status   
                
        # print("old status ",drowsiness_old_status)
        # print(EAR)
        # print(distraction_counter)
        # print(zeros_counter)
        # print(distraction_old_status)
        
        num_of_frames += 1
        
        if num_of_frames > 100:
            
            num_of_frames = 0
            
    cv2.imshow("Drowsiness Test", frame)
    cv2.imshow("New frame", new_frame)
    #cv2.resizeWindow("Drowsiness Test",300,200)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord("q"):
        break
    
    if drowsiness_old_status == full_brake_status: 
        if end_simulation_in_30_seconds == 0:
            t_end = time.time() + 5
            end_simulation_in_30_seconds = 1
        if time.time() > t_end:
            emergenecy_call()
            break    


cv2.destroyAllWindows()
cap.release()
