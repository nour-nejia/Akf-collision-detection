# Prototype embarqué - Filtre de Kalman Adaptatif pour la détection de collision
(prototype1.jpeg)

> Note importante : ce projet est un mini prototype expérimental réalisé dans un cadre académique. Ce n'est pas une voiture autonome réelle, mais une maquette permettant de tester en conditions matérielles un filtre de Kalman adaptatif appliqué à la détection de collision.

## Contexte

Ce prototype a été réalisé dans le cadre du TP d'évaluation "Signaux et Systèmes - Représentation en l'espace d'état et filtrage de Kalman" (Filière RT3, INSAT), pour la partie D consacrée à l'exploration d'une variante du filtre de Kalman couplée à l'apprentissage.

Le filtre de Kalman classique suppose que les matrices de covariance du bruit (Q et R) sont connues et constantes. Dans un contexte de détection de collision, cette hypothèse pose problème : un Q fixe réagit soit trop lentement lors d'un freinage brusque, soit trop bruyamment en phase stationnaire. Ce projet propose et implémente une solution à ce problème via un filtre de Kalman adaptatif (AKF).

## Matériel utilisé

- Carte ESP32-S3
- Accéléromètre MPU6050
- Capteur ultrason HC-SR04
- Breadboard, liaison UART vers MATLAB

## Approche

Le système estime l'état du véhicule (distance, vitesse, accélération) via un filtre de Kalman à modèle d'accélération constante. Deux versions sont comparées :

- **KF non adaptatif** : Q fixe, choisi comme compromis moyen entre réactivité et lissage.
- **KF adaptatif (AKF)** : un modèle Random Forest de régression prédit en temps réel la variance du jerk (sigma_j), utilisée pour reconstruire Q à chaque cycle en fonction du contexte de conduite détecté.

Un second modèle Random Forest, de classification cette fois, interprète les sorties du filtre pour déterminer l'état de conduite courant (stationnaire, approche, freinage, risque de collision, éloignement).

## Résultat

L'AKF s'adapte automatiquement aux changements de dynamique (freinage, approche rapide) sans réglage manuel de Q, contrairement au filtre classique. C'est cette adaptation qui constitue l'apport principal de ce prototype par rapport au TP original.

## Cadre académique

Projet réalisé par Zihar, étudiant en Génie Réseaux et Télécommunications (INSAT), dans le cadre du TP d'évaluation en Signaux et Systèmes (responsable : Rim Amara).
