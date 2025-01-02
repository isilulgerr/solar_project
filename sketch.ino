#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL

// LCD Prototypes
#define LCD_I2C_ADDR 0x27
#define LCD_BACKLIGHT   0x08// Definitions for LCD control bits
#define LCD_NOBACKLIGHT 0x00
#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit
void i2c_init(void);
void i2c_start(uint8_t address);
void i2c_write(uint8_t data);
void i2c_stop(void);
void lcd_init(uint8_t lcd_addr);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char* str);
static void lcd_send_command(uint8_t command);
static void lcd_send_data(uint8_t data);
static void lcd_write(uint8_t data, uint8_t mode);

// Buzzer Prototypes
#define BUZZER_PIN PB1
#define BUZZER_PORT PORTB
#define BUZZER_DDR DDRB
void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);


// ADC Prototypes
void adc_init(void);
uint16_t adc_read(uint8_t channel);

// Servo Prototypes
void servo_init(void);
void servo_set_position(int position);

// DHT22 Prototypes
#define DHT22_PIN PD2
void dht22_request();
uint8_t dht22_response();
uint8_t dht22_receive_data();

// Keypad Prototypes
void keypad_init();
char keypad_getkey();
// Define the keypad pins
#define ROW1 PE4 // Digital pin 2
#define ROW2 PE5 // Digital pin 3
#define ROW3 PG5 // Digital pin 4
#define ROW4 PE3 // Digital pin 5
#define COL1 PH3 // Digital pin 6
#define COL2 PH4 // Digital pin 7
#define COL3 PH5 // Digital pin 8
#define COL4 PH6 // Digital pin 9

// For test, UART Prototypes
void uart_init();
void uart_transmit(unsigned char data);

// Switch Prototypes
#define SWITCH_PIN PB4 // Digital pin 10



int main(void) {

    // Initialize peripherals
    char lcd_buffer[32];
    uint16_t max_brightness = 0;
    uint8_t max_sensor = 0;
    char key;

    // Init funcitons
    servo_init();
    adc_init();
    i2c_init();
    lcd_init(LCD_I2C_ADDR);
    buzzer_init();  
    keypad_init();

    // Welcoming the user!
    lcd_set_cursor(0, 0);
    lcd_print("Welcome user!");

    // Loop
    while (1) {
      key = keypad_getkey(); // Get the key from getKey function
      if (PINB & (1 << SWITCH_PIN)) { // Switch is high (right position) which means MANUAL MODE ON
            uart_transmit('R'); // Test switch is in the right position
            if (key != '\0') {
                switch (key) { // Set the panel in position that user decided
                    case '4':
                        servo_set_position(0);
                        _delay_ms(500);
                        break;
                    case '2':
                        servo_set_position(90);
                        _delay_ms(500);
                        break;
                    case '6':
                        servo_set_position(180);
                        _delay_ms(500);
                        break;
                    default:
                        break;
                }
            }
        } else {// Switch is low (left position) which means AUTOMATIC MODE ON
          uart_transmit('L'); // Test switch is in the left position
          uint16_t brightness[3];
          brightness[0] = adc_read(0); // Read from A0
          brightness[1] = adc_read(1); // Read from A1
          brightness[2] = adc_read(2); // Read from A2

          max_brightness = brightness[0];
          max_sensor = 0;

          for (uint8_t i = 1; i < 3; i++) { // Find the max brightness so panel can turn
            if (brightness[i] > max_brightness) {
                max_brightness = brightness[i];
                max_sensor = i;
            }
          }

        servo_set_position(max_sensor * 90);  // Position servo based on brightest sensor
        }
        _delay_ms(500);

    char buffer[80]; // Keep the datas that comes from DHT
    uint8_t humidity_high, humidity_low, temp_high, temp_low, checksum;
    dht22_request(); // Send request to the sensor
    
    if (dht22_response()) {  // If sensor responds that means we have data
        // Data:
        humidity_high = dht22_receive_data();
        humidity_low = dht22_receive_data();
        temp_high = dht22_receive_data();
        temp_low = dht22_receive_data();
        checksum = dht22_receive_data();

        // Calculating humidity and temperature
        int humidity = (humidity_high << 8) | humidity_low;
        int temperature = (temp_high << 8) | temp_low;

        // Checksum control
        if ((humidity_high + humidity_low + temp_high + temp_low) < checksum + 100) {
            // Write data information to LCD
            lcd_set_cursor(1, 0);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "Hum: %d.%d%%", humidity / 10, humidity % 10);
            lcd_print(lcd_buffer);
            lcd_set_cursor(2, 0);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "Temp: %d.%dC", temperature / 10, temperature % 10);
            lcd_print(lcd_buffer);
        } else {
            //Checksum error, buzzer is on
            buzzer_on();
            _delay_ms(500);
            buzzer_off();
        }
    } else {
        //DHT22 not responding
        _delay_ms(500);
    }
  }
    return 0;
}

// Servo initialization and control functions
void servo_init(void) {
    DDRB |= (1 << PB5);  // Set PB5 (pin 11 on Arduino Mega) as output
    TCCR1A |= (1 << WGM11) | (1 << COM1A1);
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    ICR1 = 39999;  // Set TOP value for 20ms period
    OCR1A = 3000;  // Initial pulse width for 90 degrees
}

void servo_set_position(int position) {
    uint16_t pulse_width = (position * 11) + 1000; // Calcutae the pulse width with the position that given
    OCR1A = pulse_width; 
}

// ADC - Photosensors initialization and control functions
// I2C and LCD initialization and control functions
void adc_init(void) {
    ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC and set prescaler to 128
}

uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Select ADC channel with safety mask
    ADCSRA |= (1 << ADSC); // Start single conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

// I2C and LCD initialization and control functions
void i2c_init(void) {
    TWSR = 0x00;  // Set bit rate
    TWBR = 0x47;  // Set bit rate
    TWCR = (1 << TWEN);  // Enable TWI
}

void i2c_start(uint8_t address) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);  // Send START condition
    while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set

    TWDR = address;  // Load slave address
    TWCR = (1 << TWEN) | (1 << TWINT);  // Start transmission
    while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set
}

void i2c_write(uint8_t data) {
    TWDR = data;  // Load data to TWDR register
    TWCR = (1 << TWEN) | (1 << TWINT);  // Start transmission
    while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set
}

void i2c_stop(void) {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);  // Send STOP condition
    while (TWCR & (1 << TWSTO));  // Wait until STOP condition is executed
}

// LCD functions
void lcd_init(uint8_t lcd_addr) {
    _delay_ms(50);
    lcd_send_command(0x33);  // Initialize LCD in 4-bit mode
    lcd_send_command(0x32);  // Set to 4-bit mode
    lcd_send_command(0x06);  // Cursor move direction
    lcd_send_command(0x0C);  // Turn cursor off
    lcd_send_command(0x28);  // 2 line display (this setting works for 4 lines too)
    lcd_send_command(0x01);  // Clear display
    _delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};// assign addresses are specific to the internal memory of the LCD
    lcd_send_command(0x80 | (col + row_offsets[row]));// (col + row_offsets[row]) calculates the cursor memory address
}

void lcd_print(const char* str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

static void lcd_send_command(uint8_t command) {
    lcd_write(command, 0);
}

static void lcd_send_data(uint8_t data) {
    lcd_write(data, Rs);
}

static void lcd_write(uint8_t data, uint8_t mode) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;

    i2c_start(LCD_I2C_ADDR << 1);
    i2c_write(high_nibble | mode | LCD_BACKLIGHT | En);
    i2c_write(high_nibble | mode | LCD_BACKLIGHT);
    i2c_write(low_nibble | mode | LCD_BACKLIGHT | En);
    i2c_write(low_nibble | mode | LCD_BACKLIGHT);
    i2c_stop();
}

// DHT22 initialization and control functions
void dht22_request() {
    DDRD |= (1 << DHT22_PIN); // output mode
    PORTD &= ~(1 << DHT22_PIN); // send low signal
    _delay_ms(20); // wait for 20ms 
    PORTD |= (1 << DHT22_PIN); // sen high signal
    _delay_us(40); 
    DDRD &= ~(1 << DHT22_PIN); // input mode
}

uint8_t dht22_response() {
    uint8_t response = 0;
    DDRD &= ~(1 << DHT22_PIN); // input mode
    _delay_us(40);
    
    if (!(PIND & (1 << DHT22_PIN))) {
        _delay_us(80);
        if ((PIND & (1 << DHT22_PIN))) {
            response = 1;
        }
        _delay_us(80);
    }
    return response;
}

uint8_t dht22_receive_data() {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        while(!(PIND & (1 << DHT22_PIN))); // wait high level
        _delay_us(30);
        if(PIND & (1 << DHT22_PIN)) {
            data = (data << 1) | 1; // read 1
        } else {
            data = (data << 1); // read 0
        }
        while(PIND & (1 << DHT22_PIN)); // wait high level
    }
    return data;
}

// Buzzer initialization and control functions
void buzzer_init(void) {
    BUZZER_DDR |= (1 << BUZZER_PIN); // Set buzzer pin as output
}

void buzzer_on(void) {
    BUZZER_PORT |= (1 << BUZZER_PIN); // Turn buzzer on
}

void buzzer_off(void) {
    BUZZER_PORT &= ~(1 << BUZZER_PIN); // Turn buzzer off
}

// Keypad initialization and control functions
void keypad_init() {
    DDRE |= (1<<ROW1) | (1<<ROW2) | (1<<ROW4);// Set rows as output
    DDRG |= (1<<ROW3);
    
    DDRH &= ~((1<<COL1) | (1<<COL2) | (1<<COL3) | (1<<COL4));// Set columns as input with pull-up resistors
    PORTH |= (1<<COL1) | (1<<COL2) | (1<<COL3) | (1<<COL4);

    DDRB &= ~(1<<SWITCH_PIN);// Set switch pin as input with pull-up resistor
    PORTB |= (1<<SWITCH_PIN); // Enable pull-up resistor
}

char keypad_getkey() {
    const char keymap[4][4] = {// Keymap
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

    for (uint8_t row = 0; row < 4; row++) {// Loop through rows
        PORTE |= (1 << ROW1) | (1 << ROW2) | (1 << ROW4);// Set all rows to high
        PORTG |= (1 << ROW3);
        
        switch (row) {// Set current row to low
            case 0: PORTE &= ~(1 << ROW1); break;
            case 1: PORTE &= ~(1 << ROW2); break;
            case 2: PORTG &= ~(1 << ROW3); break;
            case 3: PORTE &= ~(1 << ROW4); break;
        }
        
        if (!(PINH & (1<<COL1))) return keymap[row][0];// Check columns
        if (!(PINH & (1<<COL2))) return keymap[row][1];
        if (!(PINH & (1<<COL3))) return keymap[row][2];
        if (!(PINH & (1<<COL4))) return keymap[row][3];
    }
    return 0;
}

// UART initialization and control functions for test
void uart_init() {
    UBRR0H = 0;
    UBRR0L = 103; // 9600 baud rate with F_CPU 16MHz
    UCSR0B = (1 << TXEN0); // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void uart_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}