#include "locomotion.h"
#include "stepper.h"
#include "config.h"
#include "utils.h"

// 1.8° per step, wheel 3inch diameter
constexpr double STEPS_PER_MM = (360.0/1.8) / (M_PI * 76.2);

constexpr float WHEELBASE = 132.55; // mm

Stepper_config_t step_cfg = {
    .stepPin = 39,
    .dirPin = 38,
    .enPin = 40,
    .miStep = MICROSTEP_8,
    .stepAngle = 1.8
};

Stepper_config_t step2_cfg = {
    .stepPin = 36,
    .dirPin = 35,
    .enPin = 40,
    .miStep = MICROSTEP_8,
    .stepAngle = 1.8
};


void Locomotion::init() {
    step_left.config(&step_cfg);
    step_left.init();
    step_left.setStepsPerMm(STEPS_PER_MM);
    step_left.setSpeedMm(1000, 4000, 1000);
    step_right.config(&step2_cfg);
    step_right.init();
    step_right.setStepsPerMm(STEPS_PER_MM);
    step_right.setSpeedMm(1000, 4000, 1000);

    oldPosLeft = getPosLeft();
    oldPosRight = getPosRight();

    xTaskCreate(odometry_task, "Blinker", configMINIMAL_STACK_SIZE+1024, this, 2, NULL);
    xTaskCreate(trajectory_task, "Blinker", configMINIMAL_STACK_SIZE+1024, this, 2, NULL);

}

void Locomotion::stop() {
    step_left.stopSlow();
    step_right.stopSlow();
    waitFinishedTimeout(1000 / portTICK_PERIOD_MS);
}

void Locomotion::move(float lenght, float angle) {
    float lenght_left = lenght - angle * WHEELBASE / 2.0f;
    float lenght_right = lenght + angle * WHEELBASE / 2.0f;
    step_left.runPosMm(lenght_left);
    step_right.runPosMm(lenght_right);
}

void Locomotion::moveBlocking(float lenght, float angle) {
    move(lenght, angle);
    waitFinishedTimeout(portMAX_DELAY);
}

void Locomotion::enableSteppers(bool enable) {
    if(enable) {
        step_left.enableMotor();
    } else {  
        step_left.disableMotor();
    }
}

bool Locomotion::moving() {
    bool left_moving = step_left.getState() != motor_status::IDLE && step_left.getState() != motor_status::DISABLED;
    bool right_moving = step_right.getState() != motor_status::IDLE && step_right.getState() != motor_status::DISABLED;
    return left_moving || right_moving;
}

void Locomotion::trajectory(Position* dest, int nb_pts) {

    // Question: si la traj précédente n'est pas finie, qu'est-ce qu'on fait ?
    // - rien (on continue l'ancienne), et on retourne une erreur ?
    // - on s'arrête, puis on fait la nouvelle trajectoire ?

    traj_length = nb_pts;
    // on va éviter de faire des mallocs pour les trucs "critiques" en embarqué.
    // donc soit on réserve un tableau de Position assez grand (10 c'est déjà bien) dans Locomotion,
    // soit on stocke directement le pointeur qu'on nous donne, mais ça implique qu'il
    // doit rester valable pour toute la durée d'exécution de la boucle.
    // le plus simple c'est davoir un tableau interne assez grand et d'y copier les positions
    traj_points = (Position*)malloc(nb_pts*sizeof(int));
    for (int i=0; i<nb_pts; i++) {
        traj_points[i] = dest[i];
    }
    
    // On "demande" à faire une action
    xSemaphoreGive(traj_sem);

}

void Locomotion::odometry_task(void* arg)
{
    Locomotion* that = static_cast<Locomotion*>(arg);
    
    while(true) {
        float pos_left = that->getPosLeft();
        float pos_right = that->getPosRight();
        
        float dLeft = pos_left - that->oldPosLeft;
        float dRight = pos_right - that->oldPosRight;
        float dTheta = (dRight - dLeft)/WHEELBASE;
        float d = (dRight + dLeft)/2;
        that->oldPosLeft = pos_left;
        that->oldPosRight = pos_right;
        
        // ancienne position
        Position pos = that->getPos();

        pos.x += d * cos(pos.theta+dTheta/2);
        pos.y += d * sin(pos.theta+dTheta/2);
        pos.theta += dTheta;
        pos.theta = normalise(pos.theta);

        that->setPos(pos);

        // sleep jusqu'à la prochaine période
        vTaskDelay(ODOMETRY_PERIOD_MS / portTICK_PERIOD_MS);

    }

}

void Locomotion::trajectory_task(void* arg)
{
    Locomotion* that = static_cast<Locomotion*>(arg);

    while(true) {
        xSemaphoreTake(that->traj_sem, portMAX_DELAY);

        // Tache à faire
        // Pour chaque point -> lancer un move, waitFinishedTimeout(n ms) pour tester toute les n ms si le mouvement est fini ou s'il y a un soucis
        for (int i=0; i<that->traj_length; i++) {
            Position start_pos = that->pos;
            Position end_pos = that->traj_points[i];
            float dx = end_pos.x-start_pos.x;
            float dy = end_pos.y-start_pos.y;
            float d_theta = atan2(dy, dx) - start_pos.theta;
            float mvm_length = sqrtf(dx*dx+dy*dy);
            that->move(0, d_theta);
            that->move(mvm_length, 0);
            bool b;
            do {
                // TODO: ajouter la fonction de test d'obstacle (et la gestion d'erreur??)
                bool obstacle = false;
                
                bool is_finished = that->waitFinishedTimeout(100);
                b = !is_finished && !obstacle;
            } while(b);
            // Suivant la sortie -> stop le trajet/faire une pause? Dans les deux cas, il faut se stop
            // Idée pour pause (potentiellement au milieu entre deux points):
                // On fait une boucle qui teste au max n fois les capteurs avec un delay
                    // Si toujours soucis -> on stop le trajet
                    // Sinon, on repart à i=i-1, l'odometrie reprendra la pos actuelle et le pami continuera la trajectoire
        }
        float last_rota = that->traj_points[that->traj_length-1].theta - that->pos.theta;
        that->move(0, last_rota);

        // sleep jusqu'à la prochaine période
        vTaskDelay(ODOMETRY_PERIOD_MS / portTICK_PERIOD_MS);
    }

}

BaseType_t Locomotion::waitFinishedTimeout(TickType_t xTicksToWait) {
    return step_left.waitFinishedTimeout(xTicksToWait) && step_right.waitFinishedTimeout(xTicksToWait);
}


