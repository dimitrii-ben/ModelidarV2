<p align="center"> <img src="logo ENSEA.png" width="25%" height="auto" /> </p>

# ModéLiDAR II-Projet 1A 2023
> Reprise du projet ModéLiDAR fait en 2022 par des étudiants de l'ENSEA en 1ere année.

L’objectif de ce projet est de contrôler un robot capable de faire de la reconnaissance de différents environnements grâce à un LiDAR intégré.

## Matériels utilisés
- le châssis d’une voiture RC équipé d’une batterie et de quatre moteurs et d'une caméra,
- 1 LiDAR (Velodyne Puck-Hi-Res),
- 1 Raspberry Pi 4,
- 1 STM32 (STM32F446),
- 1 joystick,
- plusieurs cartes PCB réalisés par le groupe précédent


## Description de dépôt Git
Dans ce dépôt Git vous trouverez ;
- les fichiers KiCad de la PCB servant à alimenter les moteurs droits et gauches,
- le code à implémenter sur la STM32 afin de générer les PWM pour les moteurs,
- et le code permettant une commande à distance du ModéLiDAR à l'aide d'un joystick, il est composé de 3 noeuds
  - un nœud dédié au joystick (joy_node),
  - un nœud de communication avec la Raspberry Pi (communication_node),
  - et un nœud de traitement et envoi des commandes à la STM32 (stm32_node)

## Etudiants en charge du projet :
- BAZAIA Tancrède,
- BENOIT Dimitrii,
- CHENDEA Anton,
- FERRERI Pierre,
- GAMIETTE David,
- PETIT Alexandre

## Professeurs encadrants le projet :
- BARES Christophe,
- MARTIN Alexis

Plus d'informations dans le rapport de projet : [Rapport de projet](https://docs.google.com/document/d/1G-ZUsXAikZ85jQwoOplvorRcLATqSPql3EAao3XSJj0)
