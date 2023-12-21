#pragma once

#include "Gpios.h"
#include "PinLayout.h"
#include "TeensyStep.h"


#define TAILLE_FILE 10
#define STEP_PER_MM 6.68
#define STEPPER_MAX_ACC 10000
#define STEPPER_MAX_SPEED 4000

constexpr float RAYON_PAMI = (152/2);

extern Stepper stepper_left;
extern Stepper stepper_right;
extern StepControl controller;

enum CmdType {
    TRANSLATE,
    ROTATE,
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
        float absc;
        float ord;
        float teta_0;

    public:
     
        command_t commands[TAILLE_FILE];
        void init();
        void addCommand(command_t cmd);
        void update_commands();
        void rotate(float angle);
        void translate(float distance);
        void move(float x,float y);
        void carre(float arete);
        void tour(float rayon);
    
     
};

