#include "SerialInterface.h"

SerialInterface::SerialInterface(Controller& controller) : _arm(controller) {}

void SerialInterface::begin() {
    printMenu();
}

void SerialInterface::update() {
    if (Serial.available() > 0) {
        char choice = Serial.read();
        
        // Ignore stray newlines or carriage returns
        if (choice == '\n' || choice == '\r') return; 
        
        handleMenuSelection(choice);
        printMenu();
    }
}

void SerialInterface::printMenu() {
    Serial.println();
    Serial.println("===================================");
    Serial.println("      ROBOTIC ARM CONTROLLER       ");
    Serial.println("===================================");
    Serial.println("[1] Move Single Axis");
    Serial.println("[2] Move to Joints (Degrees)");
    Serial.println("[3] Move to Cartesian (mm)");
    Serial.println("[4] Pick (Camera Driven)");
    Serial.println("[5] Place (Fixed Location)");
    Serial.println("[6] Clamp Claw");
    Serial.println("[7] Release Claw");
    Serial.println("[8] Home Arm");
    Serial.println("===================================");
    Serial.print("Select an option: ");
}

void SerialInterface::handleMenuSelection(char selection) {
    Serial.println(selection); // Echo selection
    Serial.println();

    switch (selection) {
        case '1': {
            int axis = readIntPrompt("Enter axis (1-3): ");
            float angle = readFloatPrompt("Enter target angle (deg): ");
            
            Serial.printf("Starting moveAxis(%d, %.2f)...\n", axis, angle);
            if (_arm.moveAxis(axis, angle)) {
                _arm.waitDone();
                Serial.println("SUCCESS: Move completed.");
            } else {
                Serial.println("ERROR: Move failed (Invalid axis or angle out of bounds).");
            }
            break;
        }
        case '2': {
            JointCoord q;
            q.theta1 = readFloatPrompt("Enter Base Angle [Theta1] (deg): ");
            q.theta2 = readFloatPrompt("Enter Shoulder Angle [Theta2] (deg): ");
            q.theta3 = readFloatPrompt("Enter Elbow Angle [Theta3] (deg): ");
            
            Serial.printf("Starting moveToJoints(%.2f, %.2f, %.2f)...\n", q.theta1, q.theta2, q.theta3);
            _arm.moveToJoints(q);
            _arm.waitDone();
            Serial.println("SUCCESS: Move completed.");
            break;
        }
        case '3': {
            CartesianCoord p;
            p.x = readFloatPrompt("Enter X coordinate (mm): ");
            p.y = readFloatPrompt("Enter Y coordinate (mm): ");
            p.z = readFloatPrompt("Enter Z coordinate (mm): ");
            
            Serial.printf("Starting moveToCartesian(%.2f, %.2f, %.2f)...\n", p.x, p.y, p.z);
            if (_arm.moveToCartesian(p)) {
                _arm.waitDone();
                Serial.println("SUCCESS: Move completed.");
            } else {
                Serial.println("ERROR: Target unreachable or violates joint limits.");
            }
            break;
        }
        case '4': {
            Serial.println("Starting Pick sequence...");
            if (_arm.pick()) {
                Serial.println("SUCCESS: Pick sequence completed.");
            } else {
                Serial.println("ERROR: Pick sequence failed (No camera data or out of reach).");
            }
            break;
        }
        case '5': {
            Serial.println("Starting Place sequence...");
            _arm.place();
            Serial.println("SUCCESS: Place sequence completed.");
            break;
        }
        case '6': {
            Serial.println("Clamping claw...");
            _arm.clamp();
            Serial.println("SUCCESS: Claw clamped.");
            break;
        }
        case '7': {
            Serial.println("Releasing claw...");
            _arm.release();
            Serial.println("SUCCESS: Claw released.");
            break;
        }
        case '8': {
            Serial.println("Homing arm to (0, 0, 0)...");
            _arm.home();
            Serial.println("SUCCESS: Arm homed.");
            break;
        }
        default: {
            Serial.println("ERROR: Invalid selection. Enter 1-8.");
            break;
        }
    }
    
    // Pause briefly so the user can read the result before the menu reprints
    delay(1500);
}

// --- Input Helpers ---

void SerialInterface::clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
        delay(2); // Short delay to catch lingering characters
    }
}

float SerialInterface::readFloatPrompt(const char* prompt) {
    Serial.print(prompt);
    
    clearSerialBuffer();
    while(Serial.available() == 0) {
        delay(10); // Block and wait for user input
    }
    
    float val = Serial.parseFloat();
    Serial.println(val); // Echo back to user
    clearSerialBuffer(); // Clean up trailing newlines
    
    return val;
}

int SerialInterface::readIntPrompt(const char* prompt) {
    Serial.print(prompt);
    
    clearSerialBuffer();
    while(Serial.available() == 0) {
        delay(10);
    }
    
    int val = Serial.parseInt();
    Serial.println(val); 
    clearSerialBuffer(); 
    
    return val;
}