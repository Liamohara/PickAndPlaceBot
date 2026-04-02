#include <Arduino.h>
#include "Controller.h"
#include "SerialInterface.h"

Controller arm;
SerialInterface menu(arm);

void setup() {
    Serial.begin(115200);
    
    delay(1000);
    
    arm.begin();
    Serial.println("\nArm initialization complete.");
    
    // Launch the interactive menu
    menu.begin();
}

void loop() {
    // Non-blocking UI update
    menu.update();
}