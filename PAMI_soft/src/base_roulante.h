#define TAILLE_FILE 10


enum CmdType {
    TRANSLATE,
    ROTATE,
};

typedef struct{
    CmdType command_name;
    float value;
}command_t;

class Base_roulante {
    public:
        int cmd_a_executer = 0;
        int cmd_ecrire = 0;
        int nb_elem = 0;
        command_t commands[TAILLE_FILE];
        void addCommand(command_t cmd);
        void update_commands();
        void rotate(float angle);
        void translate(float distance);
        void move(float x,float y);
        void carre(float arete);
        void tour(float rayon);
};


class File {
    public:
        
};

