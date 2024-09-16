#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 16 15:12:00 2024

@author: jetson
"""
import cv2
from ultralytics import YOLO

# Charger le modèle YOLOv8 pour la détection de pose
model = YOLO('pose_estimator_preloaded.pt').to('cuda')

# Ouvrir la webcam
cap = cv2.VideoCapture(0)  # 0 pour la webcam par défaut

def draw_keypoints(frame, keypoints, confidences, confidence_threshold=0.5):
    """
    Dessine les keypoints sur l'image avec leurs numéros.
    
    :param frame: Image sur laquelle les keypoints sont dessinés.
    :param keypoints: Tensor des coordonnées des keypoints (x, y).
    :param confidences: Scores de confiance des keypoints.
    :param confidence_threshold: Seuil de confiance pour afficher un keypoint.
    """
   
    
    conf_list = results[0].keypoints.conf.cpu().numpy()
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

    for idx, (x, y) in enumerate(keypoints_xy):
        # Vérifier si la confiance du keypoint est suffisante
        if keypoints_conf[idx] >= confidence_threshold:
            # Dessiner un cercle sur le keypoint
            cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)  # Vert pour les keypoints
            # Ajouter le numéro du keypoint à côté
            cv2.putText(frame, f'{idx+1}', (int(x) + 10, int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # Bleu pour le texte




while True:
    # Lire une image depuis la webcam
    ret, frame = cap.read()

    if not ret:
        print("Erreur de capture vidéo.")
        break

    # Convertir l'image en format compatible avec le modèle
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Effectuer la détection de pose
    results = model(frame_rgb, verbose=False)
    
    try:
        if len(results[0].keypoints.xy)>1:
            results_double = results
        # Exemple d'utilisation
        keypoints_xy = results[0].keypoints.xy  # Récupérer les keypoints
        keypoints_conf = results[0].keypoints.conf  # Récupérer les scores de confiance
        
        # Supposons que `frame` est une image capturée par la webcam ou une vidéo
        draw_keypoints(frame, keypoints_xy, keypoints_conf)


    except Exception as e:
        print(e)
        pass

    # Annoter le frame avec les résultats
    annotated_frame = results[0].plot()

    # Afficher l'image annotée
    cv2.imshow('Pose Estimation', frame)

    # Quitter avec la touche 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()
