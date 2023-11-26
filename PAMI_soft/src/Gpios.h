#pragma once
#include <Arduino.h>

constexpr uint8_t TCA1_ADDR = 0x20;
constexpr uint8_t TCA2_ADDR = 0x21;
constexpr uint32_t TCA_INT = PA12;

constexpr uint8_t TCA_INPUT_PORT = 0x00;
constexpr uint8_t TCA_OUTPUT_PORT = 0x01;
constexpr uint8_t TCA_POL_INV = 0x02;
constexpr uint8_t TCA_CONFIG = 0x03;

class Gpios {
public:

    enum Signal {
        PUMP1,
        PUMP2,
        PUMP3,
        EV1,
        EV2,
        EV3,
        MOT1_ENABLE,
        MOT2_ENABLE,
        COLOR,
        LED_G,
        LED_B,
        TIRETTE,
        BUTTON,
        LED_R,
        FDC2,
        FDC1,
        MOT1_DIR,
        MOT1_STEP,
        MOT2_DIR,
        MOT2_STEP,
        PRESSURE_CLK,
        PRESSURE1,
        PRESSURE2,
        PRESSURE3,
        DISPLAY_CLK,
        DISPLAY_DIO,
        LED,
    };

    Gpios():tca_interrupt(false){}
    void init();
    
    void setMode(Signal signal, uint32_t mode);
    void write(Signal signal, uint32_t val);
    void toggle(Signal signal);
    int read(Signal signal);

private:
    enum Bank {
        GPIO,
        TCA1,
        TCA2,
    };

    struct PinDef
    {
        Signal signal;
        Bank bank;
        uint32_t pin;
    };
    
    PinDef getDefinition(Signal signal);
    uint8_t tcaConfigure(uint8_t addr, uint8_t config);
    uint8_t tcaWrite(uint8_t addr, uint8_t data);
    void tca1Read();

    bool tca_interrupt;
    uint8_t tca1_state;
    uint8_t tca2_state;
    
    // input=1, output=0
    static constexpr uint8_t TCA1_CONFIG = 0b11011001;
    static constexpr uint8_t TCA2_CONFIG = 0b00000000;

    static constexpr std::array<PinDef, 27> _definitions {{
        {PUMP1, TCA2, 0},
        {PUMP2, TCA2, 1},
        {PUMP3, TCA2, 2},
        {EV1, TCA2, 3},
        {EV2, TCA2, 4},
        {EV3, TCA2, 5},
        {MOT1_ENABLE, TCA2, 6},
        {MOT2_ENABLE, TCA2, 7},

        {COLOR, TCA1, 0},
        {LED_G, TCA1, 1},
        {LED_B, TCA1, 2},
        {TIRETTE, TCA1, 3},
        {BUTTON, TCA1, 4},
        {LED_R, TCA1, 5},
        {FDC2, TCA1, 6},
        {FDC1, TCA1, 7},

        {MOT1_DIR, GPIO, PB0},
        {MOT1_STEP, GPIO, PA1},
        {MOT2_DIR, GPIO, PB1},
        {MOT2_STEP, GPIO, PA0},
        {PRESSURE_CLK, GPIO, PA10},
        {PRESSURE1, GPIO, PA5},
        {PRESSURE2, GPIO, PA7},
        {PRESSURE3, GPIO, PA6},
        {DISPLAY_CLK, GPIO, PB5},
        {DISPLAY_DIO, GPIO, PB4},
        {LED, GPIO, LED_BUILTIN},
    }};


};

extern Gpios gpios;