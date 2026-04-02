#pragma once
#include <Arduino.h>
#include "Controller.h"

class SerialInterface {
public:
    SerialInterface(Controller& controller);

    // Call once in setup()
    void begin();

    // Call continuously in loop()
    void update();

private:
    Controller& _arm;

    // UI Rendering
    void printMenu();
    void handleMenuSelection(char selection);

    // Input Helpers
    float readFloatPrompt(const char* prompt);
    int readIntPrompt(const char* prompt);
    void clearSerialBuffer();
};