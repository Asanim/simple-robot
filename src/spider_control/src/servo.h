#ifndef SERVO_H
#define SERVO_H

class Servo {
public:
    Servo();
    void attach(int pin);
    void write(int value);
    int read();
    void detach();
private:
    int pin;
    int value;
};

#endif // SERVO_H
