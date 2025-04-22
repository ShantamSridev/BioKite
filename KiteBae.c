#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart1
#define BAUD_RATE 115200 
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define MAX_MESSAGE_LENGTH 300

char rx_buffer[MAX_MESSAGE_LENGTH];
int buffer_index = 0;
bool receiving = false;

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float temp;
    float rel_alt;
    float battery;
} SensorData;

void parse_sensor_data(const char* message, SensorData* data) {
    // sscanf pattern must match format from sender
    sscanf(message, "<PITCH:%f,ROLL:%f,YAW:%f,TEMP:%f,ALT:%f,BATT:%f>",
           &data->pitch,
           &data->roll,
           &data->yaw,
           &data->temp,
           &data->rel_alt,
           &data->battery);
}

int main() {
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    SensorData incomingData;

    while (true) {
        while (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);

            if (c == '<') {
                buffer_index = 0;
                receiving = true;
                rx_buffer[buffer_index++] = c;
            } else if (receiving) {
                rx_buffer[buffer_index++] = c;

                if (c == '>') {
                    rx_buffer[buffer_index] = '\0'; // Null-terminate the string
                    parse_sensor_data(rx_buffer, &incomingData);
                    
                    // Print to USB serial monitor
                    printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Temp: %.2f, Alt: %.2f, Battery: %.2f, Line: %.2f, Wind: %.2f, State: %.2f\n",
                           incomingData.pitch,
                           incomingData.roll,
                           incomingData.yaw,
                           incomingData.temp,
                           incomingData.rel_alt,
                           incomingData.battery,
                           0.0, // Placeholder for line
                           0.0, // Placeholder for wind
                           0.0); // Placeholder for state

                    receiving = false;
                    buffer_index = 0;
                } else if (buffer_index >= MAX_MESSAGE_LENGTH) {
                    // Prevent buffer overflow
                    receiving = false;
                    buffer_index = 0;
                }
            }
            // else{
            //     printf("%c", c); // Print any other characters received outside of the expected format
            // }
        }
        // if (!receiving) {
        //     // If we exit the receiving state but still have data in the buffer, reset it
        //     printf("Waiting for data...\n");
        // }
        //sleep_ms(10); // avoid busy waiting
    }
}
