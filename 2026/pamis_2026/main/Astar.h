#pragma once
#include "locomotion.h"

// on défini la map + la taille de la grille
#define LARGEUR_MAP 3000
#define LONGEUR_MAP 2000
#define TAILLE_GRILLE 50

#define GRILLE_LARGEUR (LARGEUR_MAP / TAILLE_GRILLE)
#define GRILLE_LONGEUR (LONGEUR_MAP / TAILLE_GRILLE)

void init_map();
int calcul_chemin(Position depart, Position arrive, Position *chemin_suivi);
void ajouter_obstacle_temporaire(Position pos_robot, float distance_mm);