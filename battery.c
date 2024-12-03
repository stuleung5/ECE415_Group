#include <stdio.h>

// Define the thresholds for safe operation
#define MAX_SOC 100  // Maximum State of Charge (%)
#define MIN_SOC 20   // Minimum State of Charge (%)
#define MAX_TEMP 45  // Maximum allowable temperature (degrees Celsius)
#define MIN_TEMP 0   // Minimum allowable temperature (degrees Celsius)
#define MAX_VOLTAGE 4.2  // Maximum safe cell voltage (V)
#define MIN_VOLTAGE 3.0  // Minimum safe cell voltage (V)

// Function to simulate reading the current state of charge
int readSOC() {
    // Simulate reading SoC (in percentage)
    return 50;  // Example value
}

// Function to simulate reading the battery temperature
int readTemperature() {
    // Simulate reading temperature (in Celsius)
    return 25;  // Example value
}

// Function to simulate reading the battery voltage
float readVoltage() {
    // Simulate reading battery voltage (in volts)
    return 3.7;  // Example value
}

// Function to control charging based on input parameters
void controlCharging(int SOC, int temperature, float voltage) {
    if (SOC < MIN_SOC) {
        if (temperature >= MIN_TEMP && temperature <= MAX_TEMP) {
            if (voltage >= MIN_VOLTAGE && voltage <= MAX_VOLTAGE) {
                printf("Charging: SOC is below threshold, and conditions are safe.\n");
                // Code to initiate charging
            } else {
                printf("Charging stopped: Voltage out of safe range.\n");
            }
        } else {
            printf("Charging stopped: Temperature out of safe range.\n");
        }
    } else if (SOC > MAX_SOC) {
        printf("SOC is too high, stopping charging.\n");
        // Code to stop charging
    } else {
        printf("SOC is within range, no action needed.\n");
    }
}

// Main function
int main() {
    int SOC = readSOC();
    int temperature = readTemperature();
    float voltage = readVoltage();

    printf("Current SOC: %d%%\n", SOC);
    printf("Current Temperature: %d°C\n", temperature);
    printf("Current Voltage: %.2fV\n", voltage);

    controlCharging(SOC, temperature, voltage);

    return 0;
}
