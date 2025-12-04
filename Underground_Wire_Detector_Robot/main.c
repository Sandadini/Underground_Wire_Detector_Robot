#define F_CPU 16000000UL  // 16 MHz clock speed
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// I2C (TWI) functions for AVR
void i2c_init(void) {
    TWSR = 0x00;  // No prescaler
    TWBR = 0x48;  // SCL frequency: 100 kHz (for F_CPU = 16 MHz)
}

void i2c_start(void) {
    TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWSTO) | (1<<TWEN) | (1<<TWINT);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

uint8_t i2c_read_ack(void) {
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t i2c_read_nack(void) {
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

// QMC5883L Address and Register Constants
#define QMC5883L_ADDR        0x0D
#define QMC5883L_CONTROL_REG 0x09
#define QMC5883L_OUTPUT_X_LSB 0x00

void qmc5883l_init(void) {
    i2c_start();
    i2c_write(QMC5883L_ADDR << 1);  // Write mode
    i2c_write(QMC5883L_CONTROL_REG);
    i2c_write(0x01);  // Continuous mode, 10Hz, 2G range, 512 OSR
    i2c_stop();
}

void qmc5883l_read(int* x, int* y, int* z) {
    uint8_t x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;

    i2c_start();
    i2c_write(QMC5883L_ADDR << 1);  // Write mode
    i2c_write(QMC5883L_OUTPUT_X_LSB);
    i2c_start();
    i2c_write((QMC5883L_ADDR << 1) | 1);  // Read mode

    x_lsb = i2c_read_ack();
    x_msb = i2c_read_ack();
    y_lsb = i2c_read_ack();
    y_msb = i2c_read_ack();
    z_lsb = i2c_read_ack();
    z_msb = i2c_read_nack();

    i2c_stop();

    *x = ((int16_t)x_msb << 8) | x_lsb;
    *y = ((int16_t)y_msb << 8) | y_lsb;
    *z = ((int16_t)z_msb << 8) | z_lsb;
}

// UART functions for Bluetooth communication
void UART_init(unsigned int baud) {
    unsigned int ubrr = F_CPU / 16 / baud - 1;

    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << TXEN0);  // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit
}

void UART_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));  // Wait for empty buffer
    UDR0 = data;  // Transmit data
}

void UART_transmit_string(const char *str) {
    while (*str) {
        UART_transmit(*str);
        str++;
    }
}

// Motor control functions
void motor_init() {
    // Set Enable Pins as output
    DDRB |= (1 << PB0) | (1 << PB1);
    
    // Set Motor Direction Pins as output
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
}

void move_forward() {
    // Set Enable Pins HIGH to turn on motors
    PORTB |= (1 << PB0) | (1 << PB1);
    
    // Set motor directions: IN1 = HIGH, IN2 = LOW, IN3 = HIGH, IN4 = LOW
    PORTC |= (1 << PC0);  // IN1 -> HIGH
    PORTC &= ~(1 << PC1); // IN2 -> LOW
    PORTC |= (1 << PC2);  // IN3 -> HIGH
    PORTC &= ~(1 << PC3); // IN4 -> LOW
}

void stop_motors() {
    // Turn off the Enable Pins to stop motors
    PORTB &= ~((1 << PB0) | (1 << PB1));
}

int motors_running = 0;  // Flag to indicate if motors are running
int step_count = 0;      // Number of steps taken

int main(void) {
    int x, y, z;
    char buffer[128];

    i2c_init();         // Initialize I2C
    qmc5883l_init();    // Initialize QMC5883L
    UART_init(9600);    // Initialize UART for Bluetooth
    motor_init();       // Initialize motor control pins

    while (1) {
        // Move car forward
        move_forward();
        motors_running = 1;  // Motors are now running
        _delay_ms(300);      // Move for 300 milliseconds
        step_count++;        // Increment step count with each movement

        // Stop car and allow magnetometer reading
        stop_motors();
        motors_running = 0;  // Motors are stopped
        _delay_ms(1000);     // Wait for motors to stop completely

        // Only read magnetometer if motors are stopped
        if (!motors_running) {
            qmc5883l_read(&x, &y, &z);
            // Send magnetometer data and step count
            sprintf(buffer, "Step: %d, X: %d, Y: %d, Z: %d\r\n", step_count, x, y, z);
            UART_transmit_string(buffer);  // Send data via Bluetooth
        }
    }

    return 0;
}

