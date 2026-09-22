#include "relay.h"

bool relay::ch1State = false;
bool relay::ch2State = false;

void relay::init() {
    pinMode(PIN_RELAY_CH1, OUTPUT);
    pinMode(PIN_RELAY_CH2, OUTPUT);
    openAll(); // relays start open
}

void relay::openCh1() {
    digitalWrite(PIN_RELAY_CH1, LOW);
    ch1State = false;
}

void relay::closeCh1() {
    digitalWrite(PIN_RELAY_CH1, HIGH);
    ch1State = true;
}

bool relay::isCh1Closed() {
    return ch1State;
}

void relay::openCh2() {
    digitalWrite(PIN_RELAY_CH2, LOW);
    ch2State = false;
}

void relay::closeCh2() {
    digitalWrite(PIN_RELAY_CH2, HIGH);
    ch2State = true;
}

bool relay::isCh2Closed() {
    return ch2State;
}

void relay::openAll() {
    openCh1();
    openCh2();
}

void relay::closeAll() {
    closeCh1();
    closeCh2();
}