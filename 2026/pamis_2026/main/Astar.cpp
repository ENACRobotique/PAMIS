#include "Astar.h"
#include <math.h>
#include <string.h>

// taille autour du robot marge de sécurité
#define MARGE_ROBOT_MM 0

// définition de la carte
static uint8_t map[GRILLE_LARGEUR][GRILLE_LONGEUR];

struct Noeud
{
    int parent_x;
    int parent_y;
    float g_cout; // cout depuis le depart
    float h_cout; // cout estimer juqu'a l'arriver
    float f_cout; // g_cout + h_cout
    bool ouvert;  // case découverte mais pas encore exploré
    bool fermer;  // case visité et validé
};

static Noeud noeuds[GRILLE_LARGEUR][GRILLE_LONGEUR];

int x_vers_grille(int x_mm)
{
    if (x_mm < 0)
        return 0;
    int index = x_mm / TAILLE_GRILLE;
    if (index >= GRILLE_LARGEUR)
        return GRILLE_LARGEUR - 1;
    return index;
}

int y_vers_grille(int y_mm)
{
    if (y_mm < 0)
        return 0;
    int index = y_mm / TAILLE_GRILLE;
    if (index >= GRILLE_LONGEUR)
        return GRILLE_LONGEUR - 1;
    return index;
}

void interdire_zone(int x_min, int x_max, int y_min, int y_max)
{
    int debut_x = x_vers_grille(x_min - MARGE_ROBOT_MM);
    int fin_x = x_vers_grille(x_max + MARGE_ROBOT_MM);
    int debut_y = y_vers_grille(y_min - MARGE_ROBOT_MM);
    int fin_y = y_vers_grille(y_max + MARGE_ROBOT_MM);

    // Changement de < à <= pour bien inclure la dernière case de l'obstacle
    for (int x = debut_x; x <= fin_x; x++)
    {
        for (int y = debut_y; y <= fin_y; y++)
        {
            if (x >= 0 && x < GRILLE_LARGEUR && y >= 0 && y < GRILLE_LONGEUR)
                map[x][y] = 255; // pour les zone interdite
        }
    }
}

void init_map()
{
    memset(map, 0, sizeof(map));

    interdire_zone(600, 2400, 1550, 2000); // zone du grenier

    // zone d'attente du robot avant de rentrer
    // interdire_zone(600, 1100, 1200, 1600); // RAJ
    // interdire_zone(2550, 3000, 1300, 900); // RAB

    // on interdit les gardes a manger on s'approchera sans renter entièrement dedans
    interdire_zone(0, 200, 700, 900);       // J1
    interdire_zone(700, 900, 700, 900);     // J2
    interdire_zone(600, 800, 0, 200);       // J3
    interdire_zone(1150, 1350, 1350, 1550); // J4

    interdire_zone(2800, 3000, 700, 900);   // B1
    interdire_zone(2100, 2300, 700, 900);   // B2
    interdire_zone(2200, 2400, 0, 200);     // B3
    interdire_zone(1650, 1850, 1350, 1550); // B4

    interdire_zone(1400, 1600, 700, 900); // C1
    interdire_zone(1400, 1600, 0, 200);   // C2

    // on lui interdit les murs pour pas que ça blmoque les vl5
    // interdire_zone(0, LARGEUR_MAP, -50, 0);                        // Mur du bas
    // interdire_zone(0, LARGEUR_MAP, LONGEUR_MAP, LONGEUR_MAP + 50); // Mur du haut
    interdire_zone(-50, 0, 0, LONGEUR_MAP);                        // Mur gauche
    interdire_zone(LARGEUR_MAP, LARGEUR_MAP + 50, 0, LONGEUR_MAP); // Mur droit
}

float estimer_distance(int x1, int y1, int x2, int y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void ajouter_obstacle_temporaire(Position pos_robot, float distance_mm)
{
    int obs_x = pos_robot.x + (distance_mm * cos(pos_robot.theta));
    int obs_y = pos_robot.y + (distance_mm * sin(pos_robot.theta));

    interdire_zone(obs_x - 10, obs_x + 10, obs_y - 10, obs_y + 10);

    int rx = x_vers_grille(pos_robot.x);
    int ry = y_vers_grille(pos_robot.y);

    for (int dx = -2; dx <= 2; dx++)
    {
        for (int dy = -2; dy <= 2; dy++)
        {
            if (rx + dx >= 0 && rx + dx < GRILLE_LARGEUR && ry + dy >= 0 && ry + dy < GRILLE_LONGEUR)
            {
                map[rx + dx][ry + dy] = 0; // On efface les murs autour du robot
            }
        }
    }
}

bool ligne_de_vue(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {

        if (map[x0][y0] == 255)
            return false;

        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
    return true;
}

int calcul_chemin(Position depart, Position arrive, Position *chemin_suivi)
{
    int depart_x = x_vers_grille(depart.x);
    int depart_y = y_vers_grille(depart.y);
    int arrive_x = x_vers_grille(arrive.x);
    int arrive_y = y_vers_grille(arrive.y);

    // Débouchage
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            if (depart_x + dx >= 0 && depart_x + dx < GRILLE_LARGEUR && depart_y + dy >= 0 && depart_y + dy < GRILLE_LONGEUR)
                map[depart_x + dx][depart_y + dy] = 0;
            if (arrive_x + dx >= 0 && arrive_x + dx < GRILLE_LARGEUR && arrive_y + dy >= 0 && arrive_y + dy < GRILLE_LONGEUR)
                map[arrive_x + dx][arrive_y + dy] = 0;
        }
    }

    for (int x = 0; x < GRILLE_LARGEUR; x++)
    {
        for (int y = 0; y < GRILLE_LONGEUR; y++)
        {
            noeuds[x][y].g_cout = 100000;
            noeuds[x][y].f_cout = 100000;
            noeuds[x][y].ouvert = false;
            noeuds[x][y].fermer = false;
        }
    }

    noeuds[depart_x][depart_y].g_cout = 0;
    noeuds[depart_x][depart_y].h_cout = estimer_distance(depart_x, depart_y, arrive_x, arrive_y);
    noeuds[depart_x][depart_y].f_cout = noeuds[depart_x][depart_y].h_cout;
    noeuds[depart_x][depart_y].ouvert = true;

    bool path_found = false;
    while (true)
    {
        int x_actuel = -1, y_actuel = -1;
        float f_plusbas = 1000000;

        for (int x = 0; x < GRILLE_LARGEUR; x++)
        {
            for (int y = 0; y < GRILLE_LONGEUR; y++)
            {
                if (noeuds[x][y].ouvert && noeuds[x][y].f_cout < f_plusbas)
                {
                    f_plusbas = noeuds[x][y].f_cout;
                    x_actuel = x;
                    y_actuel = y;
                }
            }
        }

        if (x_actuel == -1)
            break;
        if (x_actuel == arrive_x && y_actuel == arrive_y)
        {
            path_found = true;
            break;
        }

        noeuds[x_actuel][y_actuel].ouvert = false;
        noeuds[x_actuel][y_actuel].fermer = true;

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;
                int v_x = x_actuel + dx, v_y = y_actuel + dy;

                if (v_x < 0 || v_x >= GRILLE_LARGEUR || v_y < 0 || v_y >= GRILLE_LONGEUR)
                    continue;
                if (map[v_x][v_y] == 255 || noeuds[v_x][v_y].fermer)
                    continue;

                if (dx != 0 && dy != 0)
                {
                    if (map[x_actuel + dx][y_actuel] == 255 || map[x_actuel][y_actuel + dy] == 255)
                    {
                        continue; // ça veut dire qu y'a un mur dans la diagonale
                    }
                }

                float cout_m = (dx == 0 || dy == 0) ? 1.0f : 1.414f;
                float new_g = noeuds[x_actuel][y_actuel].g_cout + cout_m;

                if (!noeuds[v_x][v_y].ouvert || new_g < noeuds[v_x][v_y].g_cout)
                {
                    noeuds[v_x][v_y].parent_x = x_actuel;
                    noeuds[v_x][v_y].parent_y = y_actuel;
                    noeuds[v_x][v_y].g_cout = new_g;
                    noeuds[v_x][v_y].h_cout = estimer_distance(v_x, v_y, arrive_x, arrive_y);
                    noeuds[v_x][v_y].f_cout = new_g + noeuds[v_x][v_y].h_cout;
                    noeuds[v_x][v_y].ouvert = true;
                }
            }
        }
    }

    if (!path_found)
    {
        return 0;
    }

    int path_length = 0;
    int curr_x = arrive_x;
    int curr_y = arrive_y;

    Position temp_path[200];

    while (curr_x != depart_x || curr_y != depart_y)
    {
        temp_path[path_length].x = (curr_x * TAILLE_GRILLE) + (TAILLE_GRILLE / 2);
        temp_path[path_length].y = (curr_y * TAILLE_GRILLE) + (TAILLE_GRILLE / 2);
        temp_path[path_length].theta = 0;
        path_length++;

        if (path_length >= 200)
        {
            break;
        }

        int px = noeuds[curr_x][curr_y].parent_x;
        int py = noeuds[curr_x][curr_y].parent_y;
        curr_x = px;
        curr_y = py;
    }

    if (path_length > 0)
    {
        temp_path[path_length - 1].x = depart.x;
        temp_path[path_length - 1].y = depart.y;
        temp_path[0].x = arrive.x;
        temp_path[0].y = arrive.y;
    }

    if (path_length <= 2)
    {
        for (int i = 0; i < path_length; i++)
        {
            chemin_suivi[i] = temp_path[path_length - 1 - i];
        }
        return path_length;
    }

    int idx = 0;
    int current_idx = path_length - 1;

    // ajout du point de départ au chemon
    chemin_suivi[idx++] = temp_path[current_idx];

    while (current_idx > 0)
    {
        int furthest_visible = current_idx - 1;

        // on cherhce le plus loin vers l'arrivé
        for (int i = current_idx - 1; i >= 0; i--)
        {
            int x0 = x_vers_grille(temp_path[current_idx].x);
            int y0 = y_vers_grille(temp_path[current_idx].y);
            int x1 = x_vers_grille(temp_path[i].x);
            int y1 = y_vers_grille(temp_path[i].y);

            // on prend les points et on regarde si on peut faire un tt droit
            if (ligne_de_vue(x0, y0, x1, y1))
            {
                furthest_visible = i; // met a jour du poitn visible
            }
            else
            {
                break;
            }
        }
        chemin_suivi[idx++] = temp_path[furthest_visible];

        current_idx = furthest_visible;
    }

    chemin_suivi[idx - 1].theta = arrive.theta;

    return idx;
}
int gestion_point_intermediaire(Position depart, Position arrive, Position waypoints[], int nb_waypoints, Position *chemin_final)
{
    Position chemin_temp[200];
    int index_final = 0;
    Position point_actuel = depart;

    for (int i = 0; i < nb_waypoints; i++)
    {
        int pts_segment = calcul_chemin(point_actuel, waypoints[i], chemin_temp);

        if (pts_segment == 0)
            return 0;

        for (int j = 1; j < pts_segment; j++)
        {
            chemin_temp[j].theta = 0;
            chemin_final[index_final++] = chemin_temp[j];
        }
        point_actuel = waypoints[i];
    }

    int pts_dernier = calcul_chemin(point_actuel, arrive, chemin_temp);

    if (pts_dernier == 0)
        return 0;

    for (int j = 1; j < pts_dernier; j++)
    {
        if (j == pts_dernier - 1)
        {

            chemin_temp[j].theta = arrive.theta;
        }
        else
        {
            chemin_temp[j].theta = 0;
        }

        chemin_final[index_final++] = chemin_temp[j];
    }

    return index_final;
}