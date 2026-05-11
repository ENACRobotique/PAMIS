#include "locomotion.h"
#include "stepper.h"
#include "config.h"
#include "utils.h"
#include "sts3032.h"
#include "radar_vl53.h"
#include "esp_log.h"
#include "strat.h"



bool Locomotion::peutBouger(){
    int tps = xTaskGetTickCount();
    if (match_start_time == 0 || (tps-match_start_time)/portTICK_PERIOD_MS < (MATCH_TIME * 1000)){
        return true;
    }
    return false;
}

int s=1;

// 1.8° per step, wheel 3inch diameter
constexpr double STEPS_PER_MM = (360.0/1.8) / (M_PI * 71);

// constexpr float WHEELBASE = 123.2; // mm pour les fondations
constexpr float WHEELBASE = 97.8; //mm pour wallid

float longueur_caisse = 150; //mm




Locomotion locomotion;

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
    pos = {0, 0, 0};
    match_start_time = 0;

    traj_sem = xSemaphoreCreateBinary();

    step_left.config(&step_cfg);
    step_left.init();
    step_left.setStepsPerMm(STEPS_PER_MM);
    step_left.setSpeedMm(1000, 4000, 1000);
    step_right.config(&step2_cfg);
    step_right.init();
    step_right.setStepsPerMm(STEPS_PER_MM);
    step_right.setSpeedMm(1000, 4000, 1000);
    set_seuils(0,0,0,0,0);

    oldPosLeft = getPosLeft();
    oldPosRight = getPosRight();

    xTaskCreate(odometry_task, "Odometry", configMINIMAL_STACK_SIZE + 1024, this, 2, NULL);
    xTaskCreate(trajectory_task, "Trajectory", configMINIMAL_STACK_SIZE + 1024, this, 2, &traj_TaskHandle);
}

void Locomotion::resumeTrajectory()
{
    enableSteppers(true);
    if (traj_TaskHandle != NULL)
    {
        vTaskResume(traj_TaskHandle);
    }
}

void Locomotion::abortTrajectory()
{
    is_aborted = true;
    mvm_etat = IDLE_mvm;
}

void Locomotion::pauseTrajectory()
{
    if (traj_TaskHandle != NULL)
    {
        vTaskSuspend(traj_TaskHandle);
    }
}

DistancesRoues Locomotion::stop()
{
    // arret lent
    float step_to_do_left = step_left.stopSlow();
    float step_to_do_right = step_right.stopSlow();
    waitFinishedTimeout(1000 / portTICK_PERIOD_MS);
    DistancesRoues d={
        .d1=step_to_do_left,
        .d2=step_to_do_right
    };
    return d;
}


void Locomotion::move(float d, float alpha) {

    alpha=s*alpha;
    float d1=d-(WHEELBASE/2)*alpha;
    float d2=d+(WHEELBASE/2)*alpha;


    _move(d1, d2);


}

void Locomotion::_move(float d1, float d2) {
    if (!peutBouger()){
        return;
    }
    
    float a1=0;
    float a2=0;
    float v1=0;
    float v2=0;
    float d=(d1+d2)/2;
    if (d==0){
        v1=vitesse_pami;
        v2=vitesse_pami;
        a1=acceleration_pami;
        a2=acceleration_pami;}
    else{
    v1=abs(d1/d)*vitesse_pami;
    v2=abs(d2/d)*vitesse_pami;
    a1=abs(d1/d)*acceleration_pami;
    a2=abs(d2/d)*acceleration_pami;}

    step_left.setSpeedMm(a1, v1, a1);
    step_right.setSpeedMm(a2, v2, a2);
    step_left.runPosMm(d1);
    step_right.runPosMm(d2);
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
    // Comment définir si on a fini ? mettre une valeur particulière pour traj_length ou regarder l'état du déplacement (Idle, turn_final...)?

    traj_length = nb_pts;
    // on va éviter de faire des mallocs pour les trucs "critiques" en embarqué.
    // donc soit on réserve un tableau de Position assez grand (10 c'est déjà bien) dans Locomotion,
    // soit on stocke directement le pointeur qu'on nous donne, mais ça implique qu'il
    // doit rester valable pour toute la durée d'exécution de la boucle.
    // le plus simple c'est davoir un tableau interne assez grand et d'y copier les positions
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

int Locomotion::trajectory_movement() {
    // Vaut-il mieux pas garder cet indice dans Locomotion pour savoir où on en est ?
    int i=0;
    while(i<traj_length) {
        bool is_finished = waitFinishedTimeout(100);
        switch(mvm_etat) {
            case IDLE_mvm:
                {
                    mvm_etat = TURN_mvm;
                    float dx = traj_points[i].x-pos.x;
                    float dy = traj_points[i].y-pos.y;
                    float d_theta = atan2(dy, dx) - pos.theta;
                    move(0, d_theta);
                    break;
                }
            case TURN_mvm:
                {
                    if (is_finished) {
                        mvm_etat = CRUISE_mvm;
                        float dx = traj_points[i].x-pos.x;
                        float dy = traj_points[i].y-pos.y;
                        float mvm_length = sqrtf(dx*dx+dy*dy);
                        move(mvm_length, 0);
                    }
                    break;
                }
            case CRUISE_mvm:
                {
                    if (is_finished) {
                        if (i==traj_length-1) {
                            mvm_etat = TURN_FINAL_mvm;
                            float d_theta = traj_points[i].theta - pos.theta;
                            move(0, d_theta);
                        } else {
                            i++;
                            mvm_etat = IDLE_mvm;
                        }
                    }
                    break;
                }
            case TURN_FINAL_mvm:
                {
                    if (is_finished) {
                        return 1;
                    }
                }
        }
    }
    return -1;
}

void Locomotion::setMatchStart()
{
    match_start_time = xTaskGetTickCount();
}

void Locomotion::trajectory_task(void* arg)
{
    Locomotion* that = static_cast<Locomotion*>(arg);

    while(true) {
        xSemaphoreTake(that->traj_sem, portMAX_DELAY);

        // Tache à faire
        // Pour chaque point -> lancer un move, waitFinishedTimeout(n ms) pour tester toute les n ms si le mouvement est fini ou s'il y a un soucis
        that->trajectory_movement();
    }

}

BaseType_t Locomotion::waitFinishedTimeout(TickType_t xTicksToWait) {
    return step_left.waitFinishedTimeout(xTicksToWait) && step_right.waitFinishedTimeout(xTicksToWait);

}


void Locomotion::set_seuils(float a,float b,float c,float d,float e){
    seuils[0]=a;
    seuils[1]=b;
    seuils[2]=c;
    seuils[3]=d;
    seuils[4]=e;
}

bool Locomotion::danger(){
    float d0=get_distance(0);
    float d1=get_distance(1);
    float d2=get_distance(2);
    float d3=get_distance(3);
    float d4=get_distance(4);

    return (d0<= seuils[0] || d1<= seuils[1]  || d2<= seuils[2]  || d3 <= seuils[3] || d4 <= seuils[4]);
}

DistancesRoues Locomotion::leg(DistancesRoues d)
{
    _move(d.d1,d.d2);
    while(waitFinishedTimeout(100/portTICK_PERIOD_MS)!=pdTRUE){
        if (danger()){
            DistancesRoues d = stop();
            waitFinishedTimeout(portMAX_DELAY);
            return d;
        }
    }
    return {0,0};
}

void Locomotion::moveEvitement(float d,float alpha){
    float d1=d-(WHEELBASE/2)*alpha;
    float d2=d+(WHEELBASE/2)*alpha;
    DistancesRoues r={d1,d2};
    ESP_LOGI("MOVE_E", "%f  %f", d1, d2);
    while(r.d1 != 0 || r.d2 != 0){
        ESP_LOGI("MOVE_E", "d1:%f   d2:%f", r.d1, r.d2);
        r = leg(r);
        while (danger()){
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
    }
}

void Locomotion::set_speed(float v, float a)
{
    vitesse_pami=v;
    acceleration_pami=a;
}
