#include "Gpios.h"
#include <Wire.h>

Gpios gpios;

constexpr std::array<Gpios::PinDef, 27> Gpios::_definitions;

void Gpios::init() {
    Wire.begin();
    attachInterrupt(TCA_INT, [=](){tca_interrupt=true;}, FALLING);
    auto ret = tcaConfigure(TCA1_ADDR, TCA1_CONFIG);
    (void)ret;  // silent unused warning
    // delay(ret);
    ret = tcaConfigure(TCA2_ADDR, TCA2_CONFIG);
    // delay(ret);
    tca1Read();
}

Gpios::PinDef Gpios::getDefinition(Signal signal) {
    return _definitions[static_cast<size_t>(signal)];
}

void Gpios::setMode(Signal signal, uint32_t mode){
    Gpios::PinDef def = getDefinition(signal);
    switch (def.bank)
    {
    case Bank::GPIO:
        pinMode(def.pin, mode);
        break;
    case Bank::TCA1:
        // the modes are statically defined in TCA1_CONFIG
        // it could be changed dynamically if needed
        break;
    case Bank::TCA2:
        // the modes are statically defined in TCA2_CONFIG
        // configuring any of these pin in INPUT would probably be a bad idea
        break;
    
    default:
        break;
    }
}

void Gpios::write(Signal signal, uint32_t val) {
    Gpios::PinDef def = getDefinition(signal);
    uint8_t mask = (1<<def.pin);

    switch (def.bank)
    {
    case Bank::GPIO:
        digitalWrite(def.pin, val);
        break;
    case Bank::TCA1:
        if(val) {
            tca1_state |= mask;
        } else {
            tca1_state &= ~mask;
        }
        tcaWrite(TCA1_ADDR, tca1_state);
        break;
    case Bank::TCA2:
        if(val) {
            tca2_state |= mask;
        } else {
            tca2_state &= ~mask;
        }
        tcaWrite(TCA2_ADDR, tca2_state);
        break;
    
    default:
        break;
    }
}

void Gpios::toggle(Signal signal){
    Gpios::PinDef def = getDefinition(signal);
    uint8_t mask = (1<<def.pin);

    switch (def.bank)
    {
    case Bank::GPIO:
        digitalToggle(def.pin);
        break;
    case Bank::TCA1:
        if(tca1_state & mask) {
            tca1_state &= ~mask;
        } else {
            tca1_state |= mask;
        }
        tcaWrite(TCA1_ADDR, tca1_state);
        break;
    case Bank::TCA2:
        if(tca2_state & mask) {
            tca2_state &= ~mask;
        } else {
            tca2_state |= mask;
        }
        tcaWrite(TCA2_ADDR, tca2_state);
        break;
    
    default:
        break;
    }
}

int Gpios::read(Signal signal){

    if(tca_interrupt) {
        tca1Read();
        tca_interrupt = false;
    }

    Gpios::PinDef def = getDefinition(signal);
    switch (def.bank)
    {
    case Bank::GPIO:
        return digitalRead(def.pin);
        break;
    case Bank::TCA1:
        return (tca1_state & (1<<def.pin)) ? HIGH: LOW;
        break;
    case Bank::TCA2:
        return (tca2_state & (1<<def.pin)) ? HIGH: LOW;
        break;
    
    default:
        break;
    }
    return 0;
}

uint8_t Gpios::tcaConfigure(uint8_t addr, uint8_t config) {
    Wire.beginTransmission(addr);
    uint8_t buffer[] = {TCA_CONFIG, config};
    Wire.write(buffer, 2);
    return Wire.endTransmission();
}

uint8_t Gpios::tcaWrite(uint8_t addr, uint8_t data) {
    Wire.beginTransmission(addr);
    uint8_t buffer[] = {TCA_OUTPUT_PORT, data};
    Wire.write(buffer, 2);
    return Wire.endTransmission();
}

void Gpios::tca1Read() {
    Wire.beginTransmission(TCA1_ADDR);
    Wire.write(TCA_INPUT_PORT);
    Wire.endTransmission();
    Wire.requestFrom(TCA1_ADDR, static_cast<uint8_t>(1));
    tca1_state = Wire.read();
}
