void strat_grenier(void* arg);



class StratWallid{
public:
    
    void moveWallid(float d,float alpha);
    void set_team(int c, int bm, int bb);
    void sortir_caisse();
    void placer_frigo_1();
    void placer_frigo_2();
    void pousser_caisse();
    void vider_frigos();
    void vider_frigo1();
    void vider_frigo2();
    void taper_mur();
};

extern StratWallid stratWallid;
