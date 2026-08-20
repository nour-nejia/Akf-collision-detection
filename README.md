# Mini Prototype – Poursuite avec Filtre de Kalman Adaptatif (AKF)

![Prototype](./prototype.jpeg)

> **Note importante** : ce projet est un **mini prototype expérimental** réalisé dans un cadre académique. Ce n'est **pas une voiture autonome réelle**, mais une maquette destinée à illustrer et tester en conditions matérielles un algorithme de filtrage.

## Contexte

Ce prototype a été réalisé dans le cadre du TP d'évaluation *"Signaux et Systèmes – Représentation en l'espace d'état et filtrage de Kalman"* (Filière RT3, INSAT), pour la **partie D** du TP consacrée à l'exploration d'une variante du filtre de Kalman couplée à l'apprentissage.

L'objectif du TP original était la poursuite d'une source mobile via un Filtre de Kalman Étendu (EKF) classique, où l'état caché du système (position et vitesse) est estimé à partir d'observations bruitées (TOA – Time of Arrival).

Ce mini prototype transpose cette logique sur du matériel réel, avec une évolution : un **AKF (Adaptive Kalman Filter)**.

## Algorithme : AKF avec correction par IA

Contrairement à l'EKF classique où la matrice de covariance du bruit est fixée a priori, ce prototype utilise un **modèle d'IA qui corrige dynamiquement la matrice de covariance** du filtre en fonction des observations, permettant une adaptation en temps réel plutôt qu'un réglage statique des hyperparamètres.

## Matériel utilisé

- Carte **ESP32-S3**
- **Capteur ultrasonique** (type HC-SR04) – alimente le filtre AKF en observations pour la poursuite / l'estimation d'état
- Breadboard et câblage de prototypage

## Fonctionnement

1. Le capteur ultrasonique mesure la distance à la cible/source mobile.
2. Ces mesures bruitées sont injectées comme observations dans le filtre AKF.
3. Le modèle d'IA ajuste la matrice de covariance du filtre à chaque itération, en fonction des observations reçues.
4. Le filtre produit une estimation de l'état (position/vitesse) plus robuste qu'un EKF à covariance fixe.

## Cadre académique

Projet réalisé par Zihar, étudiant en Génie Réseaux et Télécommunications (INSAT), dans le cadre du TP d'évaluation en Signaux et Systèmes (responsable : Rim Amara).
