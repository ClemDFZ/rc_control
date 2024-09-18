#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 16 15:12:00 2024

@author: jetson
"""
import cv2
from threading import Thread
import os, sys
repo_root = os.getcwd()+"/../"
import time
import torch
import serial
import random
t0 = time.time()
from ultralytics import YOLO
# Charger le modèle YOLOv8 pour la détection de pose
model = YOLO(repo_root+"models/pose_estimator_preloaded.pt").to('cuda')
print("Model imported in ",int(time.time()-t0)," seconds")
# Ouvrir la webcam

FACE_LOST_TRESHOLD = 10

class RC_car(Thread):
    def __init__(self,cam_resolution = [800,600]):
        self.t0 = time.time()
        self.face_lost = True
        self.face_lost_framecount = 0
        self.serial_connected = False
        for x in [0,1]:
            try:
                port = '/dev/ttyACM'+str(x)
                self.ser = serial.Serial(port, 115200,timeout=1) 
                self.ser.dtr= False
                self.serial_connected = True
                print("Connection with arduino on port ",port)
                break
            except:
                self.ser = False
                pass
        self.serial_free = True
        self.movement_free = True
        self.ser.dtr= False
        self.servo_angle = 135
        self.servo_free = True
        self.Kp_X = (3.1416/2)/(cam_resolution[1]/2) #dX depends on cam resolution, Kp set for PI rad/s max
        self.Kp_Y = 0.03
        self.threshold_dY = 5
        self.threshold_dX = 20
        self.cap = cv2.VideoCapture(0)
        Thread(target=self.arduino_bootup_thread).start()
        Thread(target=self.movement_thread).start()
        self.resolution = cam_resolution
        self.cap.set(3,cam_resolution[0])
        self.cap.set(4,cam_resolution[1])
#        self.run()
        Thread.__init__(self)
        self.start()
        
    def log(self,msg):
        
        time
        
    def arduino_bootup_thread(self):
        t0 = time.time()
        if self.serial_connected:
            while time.time()-t0<=40:     
                if self.ser.in_waiting > 0:          
                    line = self.ser.readline().decode('utf-8').rstrip()  # Lire une ligne complète, la décoder
                    if "setup end" in line:
                        print("Arduino loop started")
                        break            
            self.update_car_movement(servo=self.servo_angle)   

        
    def set_Kp_Y(self,value):
        self.Kp_Y = value
        
    def send_serial_string(self,string,wait_for_response = True,timeout=2):       
        if self.serial_connected:
            while not self.serial_free:
                time.sleep(0.1)
            self.serial_free = False
            string+="\n"
            self.ser.write(string.encode())  # Encoder la chaîne en bytes et envoyer
            self.ser.flush()
            if wait_for_response:
                t0 = time.time()
                while time.time()-t0 <= timeout:
                    if self.ser.in_waiting > 0:  # Vérifier s'il y a des données à lire
                        line = self.ser.readline().decode('utf-8').rstrip()  # Lire une ligne complète, la décoder
                        if "done" in line:
                            break            
                    time.sleep(0.1)  # Pause légère pour éviter une boucle trop rapide
            print(string)
            self.serial_free = True
            
    def run(self):
      while True:
        # Lire une image depuis la webcam
        ret, frame = self.cap.read()
    
        if not ret:
            print("Erreur de capture vidéo.")
            break
    
        # Convertir l'image en format compatible avec le modèle
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
        # Effectuer la détection de pose
        results = model(frame_rgb, verbose=False)
        
        try:
            confidences = results[0].keypoints.conf
            keypoints = results[0].keypoints.xy
            if type(confidences)!=type(None):
                keypoints_xy, keypoints_conf = filter_keypoints(keypoints, confidences)
                dX,dY = self.head_distance(keypoints_xy,cam_res=self.resolution)
                self.center_camera(dX,dY)
                draw_keypoints(frame, keypoints_xy)
            else:
                self.add_lost_frame()
                
        
        
        except Exception as e:
            print(e)
            pass
    
        # Annoter le frame avec les résultats
        annotated_frame = results[0].plot()
    
        # Afficher l'image annotée
        cv2.imshow('Pose Estimation', frame)
    
        # Quitter avec la touche 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.release()      
            
    def add_lost_frame(self):
        self.face_lost_framecount+=1
        if self.face_lost_framecount >= FACE_LOST_TRESHOLD:
            self.face_lost = True
            self.random_cam_angle()
            
    def movement_thread(self):
        while True:
            if self.movement_free:
                time.sleep(0.1)
            else:
                str2send = ""
                if self.target_Vx!=None:
                    str2send+="X"
                    str2send+=str(self.target_Vx)
                    str2send+=" "
                if self.target_Vy!=None:
                    str2send+="Y"
                    str2send+=str(self.target_Vy)
                    str2send+=" "
                if self.target_wZ!=None:
                    str2send+="Z"
                    str2send+=str(self.target_wZ)
                    str2send+=" "
                if self.target_servo!=None:
                    str2send+="S"
                    str2send+=str(self.target_servo)
                    str2send+=" "  
                self.send_serial_string(str2send)
                if self.target_servo!=None:
                    self.servo_angle = self.target_servo
                self.movement_free = True
                   
    def update_car_movement(self,Vx=None,Vy=None,wZ=None,servo=None):  
        self.target_Vx = Vx
        self.target_Vy = Vy
        self.target_wZ = wZ
        print(self.target_wZ)
        self.target_servo = servo            
        self.movement_free = False    

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
        
    def center_camera(self,dX,dY):
        [Vx,Vy,wZ,servo] = [None,None,None,None]
        if not self.face_lost:
            if abs(dY)>=self.threshold_dY:
                servo = self.servo_angle-self.Kp_Y*dY
            if abs(dX)>=self.threshold_dX:               
                wZ = self.Kp_X*dX
            elif dX == 0:
                pass
            else:
                pass
            

        self.update_car_movement(Vx=Vx,Vy=Vy,wZ=wZ,servo=servo)
                
    def random_cam_angle(self):
        pass
        random_angle = random.randint(70,135)
        self.update_car_movement(servo=random_angle,wZ=0)
      
    def head_distance(self,keypoints,cam_res):
        nose_pose = keypoints[0]
        left_eye = keypoints[1]
        right_eye = keypoints[2]
        for keypoint in [nose_pose,left_eye,right_eye]:
            if (keypoint[0] == 0 and keypoint[1] == 0):
                self.add_lost_frame()
            else:
                self.face_lost_framecount = 0
                self.face_lost = False
        if not self.face_lost:
            avg_eye_height = (left_eye[1]+right_eye[1])//2
            middle_X = cam_res[0]//2
            middle_Y = cam_res[1]//2
            dX = middle_X-nose_pose[0]
            dY = middle_Y-avg_eye_height
        else:
            dX = 0
            dY = 0
        return dX,dY
    
        
        
def filter_keypoints(keypoints, confidences, confidence_threshold=0.5):
    """
    Filtre si plusieurs keypoints, récup celui avec le meilleur score confidence +
    + nombre de keypoints au dessus du threshold
    
    :param keypoints: Tensor des coordonnées des keypoints (x, y).
    :param confidences: Scores de confiance des keypoints.
    :param confidence_threshold: Seuil de confiance pour afficher un keypoint.
    """
    
    conf_list = confidences.cpu().numpy()

    n_person,n_point = conf_list.shape
    if n_person>1:
        #print(conf_list)
        conf_comp = [0 for x in range(n_person)]   
        for point in range(n_point):
            max_index = 0
            max_conf = conf_list[0][point]
            for person in range(n_person):
                conf = conf_list[person][point]
                if conf>confidence_threshold:
                    conf_comp[person]+=1
                if conf >= max_conf:
                    max_conf = conf
                    max_index = person
            if max_conf >=confidence_threshold:
                conf_comp[max_index]+=1
            
        #print(conf_comp)
        max_confidence_index = conf_comp.index(max(conf_comp))
        keypoints_xy = keypoints[max_confidence_index]
        keypoints_conf = confidences[max_confidence_index]
    else:
        keypoints_xy = keypoints[0]
        keypoints_conf = confidences[0]
        
    keypoints_xy = keypoints_xy.cpu().numpy()
    keypoints_conf = keypoints_conf.cpu().numpy()

    return keypoints_xy, keypoints_conf

        
def draw_keypoints(frame, keypoints):
    """
    Dessine les keypoints sur l'image avec leurs numéros.
    
    :param frame: Image sur laquelle les keypoints sont dessinés.
    :param keypoints: Tensor des coordonnées des keypoints (x, y).
    """        
    height,width,_=frame.shape
    center_x = width//2
    center_y = height//2
    thickness = 2
    color = (255,255,255)
    
    cv2.line(frame,(center_x,0),(center_x,height),color,thickness)
    cv2.line(frame,(0,center_y),(width,center_y),color,thickness)
    for idx, (x, y) in enumerate(keypoints):
        # Vérifier si la confiance du keypoint est suffisante
            # Dessiner un cercle sur le keypoint
            cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)  # Vert pour les keypoints
            # Ajouter le numéro du keypoint à côté
            cv2.putText(frame, f'{idx+1}', (int(x) + 10, int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # Bleu pour le texte



if __name__=="__main__":
    rc = RC_car()



