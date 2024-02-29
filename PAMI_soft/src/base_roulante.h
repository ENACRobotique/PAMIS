#pragma once

#include "PinLayout.h"
#include "TeensyStep.h"


#define TAILLE_FILE 100
#define STEP_PER_MM 6.68
#define STEPPER_MAX_ACC 10000
#define STEPPER_MAX_SPEED 4000
#define DISTANCE_EVITEMENT 200 
#define TAILLE_TABLE_X 2000
#define TAILLE_TABLE_Y 3000

constexpr float RAYON_PAMI = 65.142;

extern Stepper stepper_left;
extern Stepper stepper_right;
extern StepControl controller;

enum CmdType {
    TRANSLATE,
    ROTATE,
};


typedef struct{
  float x;
  float y;
  float theta;
} coord;

enum EtatRobot{
    ATTENTE_DEBUT,
    TOURNER,
    AVANCER,
    OBSTACLE_TOURNER,
    OBSTACLE_AVANCER,
    ETAT_FIN,
};



typedef struct{
    CmdType command_name;
    float value;
}command_t;



class Base_roulante {
    private:
        int cmd_a_executer;
        int cmd_ecrire;
        int nb_elem;
        coord current_coord;
        command_t commands[TAILLE_FILE];
        void translate(float distance);

    public:
        void init(coord depart);
        void rotate(float angle);
        void addCommand(command_t cmd);
        void update_commands();
        bool commands_finished();
        void move(float x,float y);
        void stop();
        void rotate_point(float x, float y);
        void translate_point(float x, float y);
        void odometry ();

        coord get_current_position(){
            return current_coord;
        }
     
};

