# Configuration de votre PC

Ce guide vous aide à configurer votre ordinateur pour exécuter l'environnement de concurrence localement et développer le code.

Vous pouvez utiliser un ordinateur / ordinateur portable local ou une machine virtuelle dans votre ordinateur ou n'importe quelle plate-forme cloud comme [Google GCP](https://cloud.google.com/free){target=_blank}, [Amazon AWS](https://aws.amazon.com/free/){target=_blank}, [Microsoft Azure](https://azure.microsoft.com/en-us/free/){target=_blank}, [Digital Ocean](https://try.digitalocean.com/freetrialoffer/){target=_blank}, etc. (tous les fournisseurs de cloud ont un plan d'essai gratuit que vous pouvez utiliser).

## Configuration requise

La configuration de la compétition doit être exécutée sur [Ubuntu](https://ubuntu.com/download){target=_blank}, une saveur de [Linux](https://en.wikipedia.org/wiki/Linux){target=_blank}. Vous aurez besoin d'un ordinateur qui a:
    
- un [gpu](https://en.wikipedia.org/wiki/graphics_processing_unit){target=_blank} dédié,
     - Les cartes Nvidia ont tendance à bien fonctionner à Ubuntu
- un processeur qui est au moins un Intel i5, ou équivalent,
- au moins 4 Go d'espace disque gratuit,
- au moins 8 Go de RAM,
- [Ubuntu focal fossa](https://releases.ubuntu.com/focal/){target=_blank} installé.

## Système opérateur
Si vous n'êtes pas déjà installé, installez ** [Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank} ** sur le système en suivant [ce guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview){target=_blank}.

!!! note
     Il est fortement recommandé d'installer [focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank} d'Ubuntu en raison de [ROS (Noetic)](http://wiki.ros.org/noetic){target=_blank} dépendance.

## Installation de ROS
Vous devez installer ROS NOetic en suivant [ce guide](http://wiki.ros.org/noetic/installation/ubuntu){target=_blank} et installer `ros-nootic-desktop-full` dans l'étape` 1.4 `du guide.
