#include "Servo.h"

Servo::Servo() : pin(-1), value(0) {}

void Servo::attach(int pin) {
    this->pin = pin;
}

void Servo::write(int value) {
    this->value = value;
}

int Servo::read() {
    return value;
}

void Servo::detach() {
    pin = -1;
}
