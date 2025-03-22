#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include <LoRa.h>
#include <TeensyTimerTool.h>
#include <PWMServo.h>

//#include <SD.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SH110X.h>


// Debug Mode
//#define DEBUG

// TOAST Mode (choose one)
// REMOTE: TOAST used for remote purposes, with switches and buttons
// CONTROL: TOAST used for controlling purposes, with valves and fire

//#define TOAST_REMOTE
#define TOAST_CONTROL

// Hardware Pins
#define RX_LED_PIN        (31)
#define TX_LED_PIN        (30)
#define BUZZER_PIN        (33)
#define STATUS_LED_PIN    (32)

#define DIO_0_PIN         (2)
#define DIO_1_PIN         (3)
#define DIO_2_PIN         (4)
#define DIO_3_PIN         (5)
#define DIO_4_PIN         (6)
#define DIO_5_PIN         (7)
#define DIO_6_PIN         (8)
#define DIO_7_PIN         (9)

#define DIO_8_PIN         (37)
#define DIO_9_PIN         (36)
#define DIO_10_PIN        (0)
#define DIO_11_PIN        (1)
#define DIO_12_PIN        (24)
#define DIO_13_PIN        (25)
#define DIO_14_PIN        (28)
#define DIO_15_PIN        (29)

#define A_0_PIN           (21)
#define A_1_PIN           (20)
#define A_2_PIN           (19)
#define A_3_PIN           (18)
#define A_4_PIN           (17)
#define A_5_PIN           (16)
#define A_6_PIN           (15)
#define A_7_PIN           (14)

#define VBAT_INT_PIN      (41)
#define VBAT_EXT_PIN      (40)

#define RX_LORA_CS_PIN    (38)
#define RX_LORA_MOSI_PIN  (26)
#define RX_LORA_MISO_PIN  (39)
#define RX_LORA_SCK_PIN   (27)
#define RX_LORA_IRQ_PIN   (23)

#define TX_LORA_CS_PIN    (10)
#define TX_LORA_MOSI_PIN  (11)
#define TX_LORA_MISO_PIN  (12)
#define TX_LORA_SCK_PIN   (13)
#define TX_LORA_IRQ_PIN   (22)

#define UART_RX_PIN       (34)
#define UART_TX_PIN       (35)

// Lora Defines
#ifdef TOAST_REMOTE
  #define LORA_TX_FREQ    (870E6)
  #define LORA_RX_FREQ    (866E6)
#endif
#ifdef TOAST_CONTROL
  #define LORA_TX_FREQ    (866E6)
  #define LORA_RX_FREQ    (870E6)
#endif
#define LORA_TX_PWR       (12)
#define LORA_SF           (7)
#define LORA_PACKET_LEN   (85)
#define TOAST_IDENTIFIER  (0xEE)
#define STATUS_PACKET     (0xAA)
#define COMMAND_PACKET    (0xBE)
#define TOAST_ID          (0)
#define PACKET_ID         (1)
#define PACKET_PAYLOAD    (2)
#define LORA_TX_PERIOD    (250) //every 250ms

// Misc Defines
#define NO_RESET          (-1)
#define DIO_QTY           (16)
#define AN_QTY            (8)
#define LORA_PULSE_SIZE   (5)
#define SERIAL_RX_BUF_SIZE (5)

// LoRa Commands
#define LORA_CMD_ABORT          (0xBB)

#define LORA_CMD_V1_ON          (0x60)
#define LORA_CMD_V1_OFF         (0x61)
#define LORA_CMD_V2A_OFF        (0x10)
#define LORA_CMD_V2A_ON         (0x11)
#define LORA_CMD_V2B_OFF        (0x90)
#define LORA_CMD_V2B_ON         (0x91)
#define LORA_CMD_V3A_OFF        (0x20)
#define LORA_CMD_V3A_ON         (0x21)
#define LORA_CMD_V3B_OFF        (0xA0)
#define LORA_CMD_V3B_ON         (0xA1)
#define LORA_CMD_V4A_OFF        (0x30)
#define LORA_CMD_V4A_ON         (0x31)
#define LORA_CMD_V4B_OFF        (0xB0)
#define LORA_CMD_V4B_ON         (0xB1)
#define LORA_CMD_V5A_OFF        (0x40)
#define LORA_CMD_V5A_ON         (0x41)
#define LORA_CMD_V5B_OFF        (0xC0)
#define LORA_CMD_V5B_ON         (0xC1)
#define LORA_CMD_V6A_OFF        (0xE0)
#define LORA_CMD_V6A_ON         (0xE1)
#define LORA_CMD_V6B_OFF        (0xF0)
#define LORA_CMD_V6B_ON         (0xF1)
#define LORA_CMD_V7_OFF         (0x70)
#define LORA_CMD_V7_ON          (0x71)

#define LORA_CMD_D2A_OFF        (0x50)
#define LORA_CMD_D2A_ON         (0x51)
#define LORA_CMD_D2B_OFF        (0xD0)
#define LORA_CMD_D2B_ON         (0xD1)

#define LORA_CMD_IGN_OFF        (0x80)
#define LORA_CMD_IGN_ON         (0x81)

#define LORA_CMD_SEQ1_START     (0xC3)
#define LORA_CMD_SEQ2_START     (0xC4)
#define LORA_CMD_SEQ3_START     (0xC5)

#define LORA_CMD_DELAY1_EDIT    (0xC6)
#define LORA_CMD_DELAY2_EDIT    (0xC7)
#define LORA_CMD_DELAY3_EDIT    (0xC8)
#define LORA_CMD_DELAY4_EDIT    (0xC9)
#define LORA_CMD_DELAY5_EDIT    (0xCA)
#define LORA_CMD_DELAY6_EDIT    (0xCB)
#define LORA_CMD_DELAY7_EDIT    (0xCC)
#define LORA_CMD_DELAY8_EDIT    (0xCD)

#define LORA_CMD_V8_OFF          (0xE2) //////////////////////////// ajouté
#define LORA_CMD_V8_ON           (0xE3) //////////////////////////// ajouté

// Pin Distribution 
#define V2A_PIN                  (DIO_0_PIN)
#define V3A_PIN                  (DIO_1_PIN)
#define V4A_PIN                  (DIO_2_PIN)
#define V5A_PIN                  (DIO_3_PIN)
#define D2A_PIN                  (DIO_4_PIN)
#define V1_PIN                   (DIO_5_PIN)
#define V7_PIN                   (DIO_6_PIN)
#define IGN_PIN                  (DIO_7_PIN)
#define V2B_PIN                  (DIO_8_PIN)
#define V3B_PIN                  (DIO_9_PIN)
#define V4B_PIN                  (DIO_10_PIN)
#define V5B_PIN                  (DIO_11_PIN)
#define D2B_PIN                  (DIO_12_PIN)
#define V8_PIN                   (DIO_14_PIN)         //////////////////////////// ajouté


#define SERVO_CLOSE              (60) // Check datasheet to know what value is for what angle
#define SERVO_IGNITION           (120)
#define SERVO_OPEN               (150)

#define CF_OPEN_DELAY            (10'000'000)  //Unit:[us]
#define SFT_IGN_DELAY            (3'000'000)
#define SFT_BURN_DELAY           (2'000'000)
#define IGNITER_ON_DELAY         (10000)

// #define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define OLED_RESET -1 
// Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_HEIGHT, SCREEN_WIDTH, &Wire, OLED_RESET);

// SDA: 18
// SCL: 19

// Function Definitions
void lora_init(void);
void GPIO_init(void);
//void oled_init(void);
void heartbeat(void);
void lora_tx_handler(void);
void lora_rx_handler(void); 
void lora_parse(uint8_t *buffer);
void status_parse(uint8_t *buffer);
void command_parse(uint8_t *buffer);
void lora_packet_build(void);
void lora_send_command(uint8_t command);
void print_to_interface(void);
void battery_level_read(void);
void analog_read(void);
//void display_update(void);
void serial_rx_handle(void);
void seq_1(void);
void seq_2(void);
void seq_3(void);
void timer1_callback(void);
void timer2_callback(void);
void timer3_callback(void);
void timer4_callback(void);
void timer5_callback(void);
void timer6_callback(void);
void timer7_callback(void);
void timer8_callback(void);
void set_dio_pin(uint8_t pin);
void clear_dio_pin(uint8_t pin);

const unsigned char PROGMEM ert_splash[] = {
// 'splash_hor', 128x64px
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x04, 0xf0, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x01, 0xff, 0xf8, 0x01, 0xff, 0xff, 0x80, 0x00, 0x0d, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x98, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x98, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x18, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x30, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x30, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 
0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0xe0, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x7f, 0xc0, 0x01, 0xb8, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x03, 0xe0, 0x10, 0xc0, 0x00, 
0x00, 0x00, 0xff, 0x80, 0x03, 0x18, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x0f, 0x60, 0x19, 0xc0, 0x00, 
0x00, 0x01, 0xff, 0x00, 0x02, 0x08, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x1c, 0x60, 0x0f, 0x00, 0x00, 
0x00, 0x01, 0xfe, 0x00, 0x02, 0x08, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x0f, 0x60, 0x07, 0x00, 0x00, 
0x00, 0x00, 0xff, 0x80, 0x03, 0x18, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x03, 0xe0, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x7f, 0xc0, 0x01, 0xb8, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x78, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x3f, 0xe0, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x03, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x78, 0x00, 0x00, 
0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x38, 0x7f, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x07, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x07, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x3f, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x01, 0xff, 0xf8, 0x03, 0xff, 0xff, 0x80, 0x00, 0x0e, 0x70, 0x78, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x07, 0xe0, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x01, 0x80, 0x7f, 0xc0, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// LoRa Variables
LoRaClass LoRa_rx;
LoRaClass LoRa_tx;

uint8_t lora_rx_buf[LORA_PACKET_LEN];
uint8_t lora_tx_buf[LORA_PACKET_LEN];
int lora_rssi;

// LOCAL
// Battery Level Variables
float v_bat_int, v_bat_ext;

// Status Variables
uint8_t dio_status[DIO_QTY] = {0};
uint16_t analog_status[AN_QTY] = {0};

// Misc Variables
uint32_t timestamp;

// REMOTE
// Battery Level Variables
float v_bat_int_remote, v_bat_ext_remote;

// Status Variables
uint8_t dio_status_remote[DIO_QTY];
uint16_t analog_status_remote[AN_QTY];

// Sequencing variables

uint32_t delay_1 = 100;
uint32_t delay_2 = 100;
uint32_t delay_3 = 100;
uint32_t delay_4 = 100;
uint32_t delay_5 = 100;
uint32_t delay_6 = 100;
uint32_t delay_7 = 100;
uint32_t delay_8 = 100;

uint8_t seq_1_active = 0;
uint8_t seq_2_active = 0;
uint8_t seq_3_active = 0;

// Remote Variables
uint32_t timestamp_remote;
int lora_rssi_remote;
uint32_t delay_1_remote;
uint32_t delay_2_remote;
uint32_t delay_3_remote;
uint32_t delay_4_remote;
uint32_t delay_5_remote;
uint32_t delay_6_remote;
uint32_t delay_7_remote;
uint32_t delay_8_remote;
uint8_t seq_1_active_remote;
uint8_t seq_2_active_remote;
uint8_t seq_3_active_remote;

TeensyTimerTool::OneShotTimer timer_1;
TeensyTimerTool::OneShotTimer timer_2;
TeensyTimerTool::OneShotTimer timer_3;
TeensyTimerTool::OneShotTimer timer_4;
TeensyTimerTool::OneShotTimer timer_5;
TeensyTimerTool::OneShotTimer timer_6;
TeensyTimerTool::OneShotTimer timer_7;
TeensyTimerTool::OneShotTimer timer_8;

// Misc Variables
uint8_t serial_rx_byte = 0;
uint8_t serial_rx_buf[SERIAL_RX_BUF_SIZE];
uint32_t lora_tx_counter = 0;

// Servo variables
PWMServo V6A;
PWMServo V6B;

void setup() {
  GPIO_init();
  Serial.begin(9600);
  delay(1000);

  lora_init();

  // Bootup Tone
  tone(BUZZER_PIN, 3800, 50);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(50);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(50);

  tone(BUZZER_PIN, 3800, 50);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(50);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(400);

  tone(BUZZER_PIN, 3800, 500);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(500);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(300);
}

void loop() {
  timestamp = millis();
  battery_level_read();
  analog_read();

  uint32_t current_millis = millis();
  if((current_millis - lora_tx_counter) >= LORA_TX_PERIOD) {
    lora_tx_counter = current_millis;
    loraTxHandler();
  }
  
  lora_rx_handler();
  heartbeat();
}

void GPIOInit(){
  pinMode(RX_LED_PIN, OUTPUT);
  pinMode(TX_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(DIO_0_PIN, OUTPUT);
  pinMode(DIO_1_PIN, OUTPUT);
  pinMode(DIO_2_PIN, OUTPUT);
  pinMode(DIO_3_PIN, OUTPUT);
  pinMode(DIO_4_PIN, OUTPUT);
  pinMode(DIO_5_PIN, OUTPUT);
  pinMode(DIO_6_PIN, OUTPUT);
  pinMode(DIO_7_PIN, OUTPUT);
  pinMode(A_0_PIN, INPUT_PULLDOWN);
  pinMode(A_1_PIN, INPUT_PULLDOWN);
  pinMode(A_4_PIN, INPUT_PULLDOWN);
  pinMode(A_5_PIN, INPUT_PULLDOWN);
  pinMode(A_6_PIN, INPUT_PULLDOWN);
  pinMode(A_7_PIN, INPUT_PULLDOWN);
  pinMode(DIO_14_PIN, OUTPUT);      //////////////////////////// ajouté

  digitalWrite(RX_LED_PIN, LOW);
  digitalWrite(TX_LED_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(DIO_0_PIN, LOW);
  digitalWrite(DIO_1_PIN, LOW);
  digitalWrite(DIO_2_PIN, LOW);
  digitalWrite(DIO_3_PIN, LOW);
  digitalWrite(DIO_4_PIN, LOW);
  digitalWrite(DIO_5_PIN, LOW);
  digitalWrite(DIO_6_PIN, LOW);
  digitalWrite(DIO_7_PIN, LOW);
  digitalWrite(DIO_14_PIN, LOW);      //////////////////////////// ajouté
}

void loraInit(){
  SPI1.setMISO(RX_LORA_MISO_PIN);
  SPI1.setMOSI(RX_LORA_MOSI_PIN);
  SPI1.setSCK(RX_LORA_SCK_PIN);

  LoRa_rx.setPins(RX_LORA_CS_PIN, NO_RESET, RX_LORA_IRQ_PIN);
  LoRa_rx.setSPI(SPI1);
  if (!LoRa_rx.begin(LORA_RX_FREQ)) {
    while (1);
  }
  LoRa_rx.setTxPower(LORA_TX_PWR);
  LoRa_rx.setSpreadingFactor(LORA_SF);
  LoRa_rx.receive();
  LoRa_rx.onReceive(lora_rx_handler);

  LoRa_tx.setPins(TX_LORA_CS_PIN, NO_RESET,TX_LORA_IRQ_PIN);
  LoRa_tx.setSPI(SPI);
  if (!LoRa_tx.begin(LORA_TX_FREQ)) {
    while (1);
  }
  LoRa_tx.setTxPower(LORA_TX_PWR);
  LoRa_tx.setSpreadingFactor(LORA_SF);
}

void heartbeat(){
    digitalWrite(STATUS_LED_PIN, HIGH);
    tone(BUZZER_PIN, 3000, 100);
    delay(100);
    tone(BUZZER_PIN, 4000, 100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
}

void loraTxHandler(){
  loraPacketBuild();
  digitalWrite(TX_LED_PIN, HIGH);
  LoRa_tx.beginPacket();
  LoRa_tx.write(TOAST_IDENTIFIER); // TOAST
  LoRa_tx.write(STATUS_PACKET); // Packet ID
  for(int i=0;i<LORA_PACKET_LEN;i++){
    LoRa_tx.write(lora_tx_buf[i]);
  }
  LoRa_tx.endPacket(true);
  digitalWrite(TX_LED_PIN, LOW);
}

int loraRxBuf[255];

void loraRxHandler(){
  memset(loraRxBuf, 0, sizeof(loraRxBuf));
  int packetSize = LoRa_rx.parsePacket();
  if (packetSize) {
    digitalWrite(RX_LED_PIN, HIGH);
    for(int i = 0; i < packetSize; i++) {
      loraRxBuf[i] = LoRa_rx.read();
    }
    // Verify if it is a TOAST packet
    if (loraRxBuf[0] == TOAST_IDENTIFIER) {
      loraParse(loraRxBuf);
    }
    digitalWrite(RX_LED_PIN, LOW);
  }
}

void loraParse(uint8_t *buffer){
  switch (buffer[PACKET_ID])
  {
  case STATUS_PACKET:
    statusParse(buffer);
    break;
  case COMMAND_PACKET:
    commandParse(buffer);
    break;
  default:
    break;
  }
}


void status_parse(uint8_t *buffer){
  memcpy(&timestamp_remote, &(buffer[PACKET_PAYLOAD+0]), sizeof(uint32_t));
  dio_status_remote[0] = buffer[PACKET_PAYLOAD+4];
  dio_status_remote[1] = buffer[PACKET_PAYLOAD+5];
  dio_status_remote[2] = buffer[PACKET_PAYLOAD+6];
  dio_status_remote[3] = buffer[PACKET_PAYLOAD+7];
  dio_status_remote[4] = buffer[PACKET_PAYLOAD+8];
  dio_status_remote[5] = buffer[PACKET_PAYLOAD+9];
  dio_status_remote[6] = buffer[PACKET_PAYLOAD+10];
  dio_status_remote[7] = buffer[PACKET_PAYLOAD+11];
  dio_status_remote[8] = buffer[PACKET_PAYLOAD+12];
  dio_status_remote[9] = buffer[PACKET_PAYLOAD+13];
  dio_status_remote[10] = buffer[PACKET_PAYLOAD+14];
  dio_status_remote[11] = buffer[PACKET_PAYLOAD+15];
  dio_status_remote[12] = buffer[PACKET_PAYLOAD+16];
  dio_status_remote[13] = buffer[PACKET_PAYLOAD+17];
  dio_status_remote[14] = buffer[PACKET_PAYLOAD+18];
  dio_status_remote[15] = buffer[PACKET_PAYLOAD+19];
  memcpy(&analog_status_remote[0], &(buffer[PACKET_PAYLOAD+20]), sizeof(uint16_t));
  memcpy(&analog_status_remote[1], &(buffer[PACKET_PAYLOAD+22]), sizeof(uint16_t));
  memcpy(&analog_status_remote[2], &(buffer[PACKET_PAYLOAD+24]), sizeof(uint16_t));
  memcpy(&analog_status_remote[3], &(buffer[PACKET_PAYLOAD+26]), sizeof(uint16_t));
  memcpy(&analog_status_remote[4], &(buffer[PACKET_PAYLOAD+28]), sizeof(uint16_t));
  memcpy(&analog_status_remote[5], &(buffer[PACKET_PAYLOAD+30]), sizeof(uint16_t));
  memcpy(&analog_status_remote[6], &(buffer[PACKET_PAYLOAD+32]), sizeof(uint16_t));
  memcpy(&analog_status_remote[7], &(buffer[PACKET_PAYLOAD+34]), sizeof(uint16_t));
  memcpy(&v_bat_int_remote, &(buffer[PACKET_PAYLOAD+36]), sizeof(float));
  memcpy(&v_bat_ext_remote, &(buffer[PACKET_PAYLOAD+40]), sizeof(float));
  memcpy(&lora_rssi_remote, &(buffer[PACKET_PAYLOAD+44]), sizeof(int));
  memcpy(&delay_1_remote, &(buffer[PACKET_PAYLOAD+48]), sizeof(uint32_t));
  memcpy(&delay_2_remote, &(buffer[PACKET_PAYLOAD+52]), sizeof(uint32_t));
  memcpy(&delay_3_remote, &(buffer[PACKET_PAYLOAD+56]), sizeof(uint32_t));
  memcpy(&delay_4_remote, &(buffer[PACKET_PAYLOAD+60]), sizeof(uint32_t));
  memcpy(&delay_5_remote, &(buffer[PACKET_PAYLOAD+64]), sizeof(uint32_t));
  memcpy(&delay_6_remote, &(buffer[PACKET_PAYLOAD+68]), sizeof(uint32_t));
  memcpy(&delay_7_remote, &(buffer[PACKET_PAYLOAD+72]), sizeof(uint32_t));
  memcpy(&delay_8_remote, &(buffer[PACKET_PAYLOAD+76]), sizeof(uint32_t));
  seq_1_active_remote = buffer[PACKET_PAYLOAD+80];
  seq_2_active_remote = buffer[PACKET_PAYLOAD+81];
  seq_3_active_remote = buffer[PACKET_PAYLOAD+82];
}

void command_parse(uint8_t *buffer){
  switch(buffer[PACKET_PAYLOAD]){
    // DIO Commands
    case LORA_CMD_V1_OFF:
      clear_dio_pin(V1_PIN);
      break;
    case LORA_CMD_V1_ON:
      set_dio_pin(V1_PIN);
      break;
    case LORA_CMD_V2A_OFF:
      clear_dio_pin(V2A_PIN);
      break;
    case LORA_CMD_V2A_ON:
      set_dio_pin(V2A_PIN);
      break;
    case LORA_CMD_V3A_OFF:
      clear_dio_pin(V3A_PIN);
      break;
    case LORA_CMD_V3A_ON:
      set_dio_pin(V3A_PIN);
      break;
    case LORA_CMD_V4A_OFF:
      V6A.write(SERVO_CLOSE);
      dio_status[2] = 0;
      //clear_dio_pin(V4A_PIN);
      break;
    case LORA_CMD_V4A_ON:
      V6A.write(SERVO_OPEN);
      dio_status[2] = 1;
      //set_dio_pin(V4A_PIN);
      break;
    case LORA_CMD_V5A_OFF:
      //V6A.write(SERVO_CLOSE);
      //dio_status[3] = 0;
      clear_dio_pin(V5A_PIN);
      break;
    case LORA_CMD_V5A_ON:
      //V6A.write(SERVO_IGNITION);
      //dio_status[3] = 1;
      set_dio_pin(V5A_PIN);
      break;
    case LORA_CMD_V2B_OFF:
      clear_dio_pin(V2B_PIN);
      break;
    case LORA_CMD_V2B_ON:
      set_dio_pin(V2B_PIN);
      break;
    case LORA_CMD_V3B_OFF:
      clear_dio_pin(V3B_PIN);
      break;
    case LORA_CMD_V3B_ON:
      set_dio_pin(V3B_PIN);
      break;
    case LORA_CMD_V4B_OFF:
      //V6B.write(SERVO_CLOSE);
      //dio_status[10] = 0;
      clear_dio_pin(V4B_PIN);
      break;
    case LORA_CMD_V4B_ON:
      //V6B.write(SERVO_OPEN);
      //dio_status[10] = 1;
      set_dio_pin(V4B_PIN);
      break;
    case LORA_CMD_V5B_OFF:
      //V6B.write(SERVO_CLOSE);
      //dio_status[11] = 0;
      clear_dio_pin(V5B_PIN);
      break;
    case LORA_CMD_V5B_ON:
      //V6B.write(SERVO_IGNITION);
      //dio_status[11] = 1;
      set_dio_pin(V5B_PIN);
      break;
    case LORA_CMD_V6A_OFF:
      //TODO
      break;
    case LORA_CMD_V6A_ON:
      //TODO
      break;
    case LORA_CMD_V6B_OFF:
      //TODO
      break;
    case LORA_CMD_V6B_ON:
      //TODO
      break;
    case LORA_CMD_V7_OFF:
      clear_dio_pin(V7_PIN);
      break;
    case LORA_CMD_V7_ON:
      set_dio_pin(V7_PIN);
      break;
    case LORA_CMD_D2A_OFF:
      clear_dio_pin(D2A_PIN);
      break;
    case LORA_CMD_D2A_ON:
      set_dio_pin(D2A_PIN);
      break;
    case LORA_CMD_D2B_OFF:
      clear_dio_pin(D2B_PIN);
      break;
    case LORA_CMD_D2B_ON:
      set_dio_pin(D2B_PIN);
      break;
    case LORA_CMD_IGN_OFF:
      clear_dio_pin(IGN_PIN);
      break;
    case LORA_CMD_IGN_ON:
      set_dio_pin(IGN_PIN);
      break;
    case LORA_CMD_ABORT: //Abort
      clear_dio_pin(V1_PIN);
      clear_dio_pin(V2A_PIN);
      clear_dio_pin(V3A_PIN);
      //clear_dio_pin(V4A_PIN);
      clear_dio_pin(V5A_PIN);
      clear_dio_pin(V2B_PIN);
      clear_dio_pin(V3B_PIN);
      clear_dio_pin(V4B_PIN);
      clear_dio_pin(V5B_PIN);
      clear_dio_pin(V7_PIN);
      clear_dio_pin(D2A_PIN);
      clear_dio_pin(D2B_PIN);
      clear_dio_pin(IGN_PIN);
      //TODO add servo close
      break;
    case LORA_CMD_SEQ1_START: //Launch Seq 1
      seq_1();
      break;
    case LORA_CMD_SEQ2_START: //Launch Seq 2
      seq_2();
      break;
    case LORA_CMD_SEQ3_START: //Launch Seq 3
      seq_3();
      break;
    case LORA_CMD_DELAY1_EDIT: //Edit Delay 1
      //delay_1 = (buffer[3] << 8 | buffer[4] << 8 | buffer[5] << 8 | buffer[6]);
      memcpy(&delay_1, &(buffer[3]), sizeof(uint32_t));
      //Serial.println(delay_1);
      break;
    case LORA_CMD_DELAY2_EDIT: //Edit Delay 2
      memcpy(&delay_2, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_DELAY3_EDIT: //Edit Delay 3
      memcpy(&delay_3, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_DELAY4_EDIT: //Edit Delay 4
      memcpy(&delay_4, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_DELAY5_EDIT: //Edit Delay 5
      memcpy(&delay_5, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_DELAY6_EDIT: //Edit Delay 6
      memcpy(&delay_6, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_DELAY7_EDIT: //Edit Delay 7
      memcpy(&delay_7, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_DELAY8_EDIT: //Edit Delay 8
      memcpy(&delay_8, &(buffer[3]), sizeof(uint32_t));
      break;
    case LORA_CMD_V8_OFF:
      clear_dio_pin(V8_PIN);
    break;
    case LORA_CMD_V8_ON:
      set_dio_pin(V8_PIN);
    break;
    }
  }

void lora_packet_build(void){

uint8_t temp_buffer[sizeof(float)];

  // Timestamp
  memcpy(temp_buffer, &(timestamp), sizeof(uint32_t));
  lora_tx_buf[0] = temp_buffer[0];
  lora_tx_buf[1] = temp_buffer[1];
  lora_tx_buf[2] = temp_buffer[2];
  lora_tx_buf[3] = temp_buffer[3];

  // DIO States
  lora_tx_buf[4] = dio_status[0];
  lora_tx_buf[5] = dio_status[1];
  lora_tx_buf[6] = dio_status[2];
  lora_tx_buf[7] = dio_status[3];
  lora_tx_buf[8] = dio_status[4];
  lora_tx_buf[9] = dio_status[5];
  lora_tx_buf[10] = dio_status[6];
  lora_tx_buf[11] = dio_status[7];
  lora_tx_buf[12] = dio_status[8];
  lora_tx_buf[13] = dio_status[9];
  lora_tx_buf[14] = dio_status[10];
  lora_tx_buf[15] = dio_status[11];
  lora_tx_buf[16] = dio_status[12];
  lora_tx_buf[17] = dio_status[13];
  lora_tx_buf[18] = dio_status[14];
  lora_tx_buf[19] = dio_status[15];

  // AN Values
  memcpy(temp_buffer, &(analog_status[0]), sizeof(uint16_t));
  lora_tx_buf[20] = temp_buffer[0];
  lora_tx_buf[21] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[1]), sizeof(uint16_t));
  lora_tx_buf[22] = temp_buffer[0];
  lora_tx_buf[23] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[2]), sizeof(uint16_t));
  lora_tx_buf[24] = temp_buffer[0];
  lora_tx_buf[25] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[3]), sizeof(uint16_t));
  lora_tx_buf[26] = temp_buffer[0];
  lora_tx_buf[27] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[4]), sizeof(uint16_t));
  lora_tx_buf[28] = temp_buffer[0];
  lora_tx_buf[29] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[5]), sizeof(uint16_t));
  lora_tx_buf[30] = temp_buffer[0];
  lora_tx_buf[31] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[6]), sizeof(uint16_t));
  lora_tx_buf[32] = temp_buffer[0];
  lora_tx_buf[33] = temp_buffer[1];
  memcpy(temp_buffer, &(analog_status[7]), sizeof(uint16_t));
  lora_tx_buf[34] = temp_buffer[0];
  lora_tx_buf[35] = temp_buffer[1];

  // Battery Levels
  memcpy(temp_buffer, &(v_bat_int), sizeof(float));
  lora_tx_buf[36] = temp_buffer[0];
  lora_tx_buf[37] = temp_buffer[1];
  lora_tx_buf[38] = temp_buffer[2];
  lora_tx_buf[39] = temp_buffer[3];
  memcpy(temp_buffer, &(v_bat_ext), sizeof(float));
  lora_tx_buf[40] = temp_buffer[0];
  lora_tx_buf[41] = temp_buffer[1];
  lora_tx_buf[42] = temp_buffer[2];
  lora_tx_buf[43] = temp_buffer[3];

  // LoRa Info
  memcpy(temp_buffer, &(lora_rssi), sizeof(int));
  lora_tx_buf[44] = temp_buffer[0];
  lora_tx_buf[45] = temp_buffer[1];
  lora_tx_buf[46] = temp_buffer[2];
  lora_tx_buf[47] = temp_buffer[3];

  // Sequencing Info
  memcpy(temp_buffer, &(delay_1), sizeof(uint32_t));
  lora_tx_buf[48] = temp_buffer[0];
  lora_tx_buf[49] = temp_buffer[1];
  lora_tx_buf[50] = temp_buffer[2];
  lora_tx_buf[51] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_2), sizeof(uint32_t));
  lora_tx_buf[52] = temp_buffer[0];
  lora_tx_buf[53] = temp_buffer[1];
  lora_tx_buf[54] = temp_buffer[2];
  lora_tx_buf[55] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_3), sizeof(uint32_t));
  lora_tx_buf[56] = temp_buffer[0];
  lora_tx_buf[57] = temp_buffer[1];
  lora_tx_buf[58] = temp_buffer[2];
  lora_tx_buf[59] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_4), sizeof(uint32_t));
  lora_tx_buf[60] = temp_buffer[0];
  lora_tx_buf[61] = temp_buffer[1];
  lora_tx_buf[62] = temp_buffer[2];
  lora_tx_buf[63] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_5), sizeof(uint32_t));
  lora_tx_buf[64] = temp_buffer[0];
  lora_tx_buf[65] = temp_buffer[1];
  lora_tx_buf[66] = temp_buffer[2];
  lora_tx_buf[67] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_6), sizeof(uint32_t));
  lora_tx_buf[68] = temp_buffer[0];
  lora_tx_buf[69] = temp_buffer[1];
  lora_tx_buf[70] = temp_buffer[2];
  lora_tx_buf[71] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_7), sizeof(uint32_t));
  lora_tx_buf[72] = temp_buffer[0];
  lora_tx_buf[73] = temp_buffer[1];
  lora_tx_buf[74] = temp_buffer[2];
  lora_tx_buf[75] = temp_buffer[3];
  memcpy(temp_buffer, &(delay_8), sizeof(uint32_t));
  lora_tx_buf[76] = temp_buffer[0];
  lora_tx_buf[77] = temp_buffer[1];
  lora_tx_buf[78] = temp_buffer[2];
  lora_tx_buf[79] = temp_buffer[3];

  lora_tx_buf[80] = seq_1_active;
  lora_tx_buf[81] = seq_2_active;
  lora_tx_buf[82] = seq_3_active;

  #ifdef TOAST_DEBUG
  Serial.println("=== Debug print ===");
  Serial.print("Timestamp: ");
  Serial.println(timestamp);
  Serial.print("DIO States: ");
  for (uint8_t i = 0; i < DIO_QTY; i++)
  {
    Serial.print(dio_status[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("AN Values: ");
  for (uint8_t i = 0; i < AN_QTY; i++)
  {
    Serial.print(analog_status[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("Battery Voltages - INT EXT: ");
  Serial.print(v_bat_int);
  Serial.print(" ");
  Serial.println(v_bat_ext);
  Serial.print("LoRa RSSI: ");
  Serial.println(LoRa_rx.packetRssi());
  Serial.print("LoRa SNR: ");
  Serial.println(LoRa_rx.packetSnr());
  Serial.print("LoRa Packet Frequency Error: ");
  Serial.println(LoRa_rx.packetFrequencyError());
  #endif
}

void lora_send_command(uint8_t command){
  //uint32_t i=0;
  digitalWrite(TX_LED_PIN, HIGH);
  //Serial.println(command);
  LoRa_tx.beginPacket();
  LoRa_tx.write(TOAST_IDENTIFIER); // TOAST
  LoRa_tx.write(COMMAND_PACKET); // Packet ID
  // while(serial_rx_buf[i] != 0){
  for(uint8_t i = 0; i < SERIAL_RX_BUF_SIZE; i++){
    LoRa_tx.write(serial_rx_buf[i]);
    //Serial.print(serial_rx_buf[i]);
  }
  LoRa_tx.endPacket(true);
  digitalWrite(TX_LED_PIN, LOW);
}

void print_to_interface(void){
  Serial.print(timestamp);
  Serial.print(" ; ");
  Serial.print(dio_status[0]);
  Serial.print(" ; ");
  Serial.print(dio_status[1]);
  Serial.print(" ; ");
  Serial.print(dio_status[2]);
  Serial.print(" ; ");
  Serial.print(dio_status[3]);
  Serial.print(" ; ");
  Serial.print(dio_status[4]);
  Serial.print(" ; ");
  Serial.print(dio_status[5]);
  Serial.print(" ; ");
  Serial.print(dio_status[6]);
  Serial.print(" ; ");
  Serial.print(dio_status[7]);
  Serial.print(" ; ");
  Serial.print(dio_status[8]);
  Serial.print(" ; ");
  Serial.print(dio_status[9]);
  Serial.print(" ; ");
  Serial.print(dio_status[10]);
  Serial.print(" ; ");
  Serial.print(dio_status[11]);
  Serial.print(" ; ");
  Serial.print(dio_status[12]);
  Serial.print(" ; ");
  Serial.print(dio_status[13]);
  Serial.print(" ; ");
  Serial.print(dio_status[14]);
  Serial.print(" ; ");
  Serial.print(dio_status[15]);
  Serial.print(" ; ");
  Serial.print(analog_status[0]);
  Serial.print(" ; ");
  Serial.print(analog_status[1]);
  Serial.print(" ; ");
  Serial.print(analog_status[2]);
  Serial.print(" ; ");
  Serial.print(analog_status[3]);
  Serial.print(" ; ");
  Serial.print(analog_status[4]);
  Serial.print(" ; ");
  Serial.print(analog_status[5]);
  Serial.print(" ; ");
  Serial.print(analog_status[6]);
  Serial.print(" ; ");
  Serial.print(analog_status[7]);
  Serial.print(" ; ");
  Serial.print(v_bat_int);
  Serial.print(" ; ");
  Serial.print(v_bat_ext);
  Serial.print(" ; ");
  Serial.print(lora_rssi);
  Serial.print(" ; ");
  Serial.print(delay_1); // delay_1
  Serial.print(" ; ");
  Serial.print(delay_2);
  Serial.print(" ; ");
  Serial.print(delay_3);
  Serial.print(" ; ");
  Serial.print(delay_4);
  Serial.print(" ; ");
  Serial.print(delay_5);
  Serial.print(" ; ");
  Serial.print(delay_6);
  Serial.print(" ; ");
  Serial.print(delay_7);
  Serial.print(" ; ");
  Serial.print(delay_8);
  Serial.print(" ; ");
  Serial.print(seq_1_active);
  Serial.print(" ; ");
  Serial.print(seq_2_active);
  Serial.print(" ; ");
  Serial.print(seq_3_active);
  Serial.print(" ; ");
  Serial.print(timestamp_remote);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[0]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[1]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[2]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[3]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[4]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[5]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[6]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[7]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[8]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[9]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[10]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[11]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[12]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[13]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[14]);
  Serial.print(" ; ");
  Serial.print(dio_status_remote[15]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[0]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[1]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[2]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[3]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[4]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[5]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[6]);
  Serial.print(" ; ");
  Serial.print(analog_status_remote[7]);
  Serial.print(" ; ");
  Serial.print(v_bat_int_remote);
  Serial.print(" ; ");
  Serial.print(v_bat_ext_remote);
  Serial.print(" ; ");
  Serial.print(lora_rssi_remote);
  Serial.print(" ; ");
  Serial.print(delay_1_remote);
  Serial.print(" ; ");
  Serial.print(delay_2_remote);
  Serial.print(" ; ");
  Serial.print(delay_3_remote);
  Serial.print(" ; ");
  Serial.print(delay_4_remote);
  Serial.print(" ; ");
  Serial.print(delay_5_remote);
  Serial.print(" ; ");
  Serial.print(delay_6_remote);
  Serial.print(" ; ");
  Serial.print(delay_7_remote);
  Serial.print(" ; ");
  Serial.print(delay_8_remote);
  Serial.print(" ; ");
  Serial.print(seq_1_active_remote);
  Serial.print(" ; ");
  Serial.print(seq_2_active_remote);
  Serial.print(" ; ");
  Serial.print(seq_3_active_remote);
  Serial.println(" ; ");
}

void battery_level_read(void){
  v_bat_int = analogRead(VBAT_INT_PIN)*78.0/27.0*3.3/1024;
  v_bat_ext = analogRead(VBAT_EXT_PIN)*56.1/5.1*3.3/1024+0.3;
}

void analog_read(void){
  analog_status[0] = analogRead(A_0_PIN);
  analog_status[1] = analogRead(A_1_PIN);
  analog_status[2] = analogRead(A_2_PIN);
  analog_status[3] = analogRead(A_3_PIN);
  analog_status[4] = analogRead(A_4_PIN);
  analog_status[5] = analogRead(A_5_PIN);
  analog_status[6] = analogRead(A_6_PIN);
  analog_status[7] = analogRead(A_7_PIN);
}

// void display_update(){
//   display.setRotation(0);
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SH110X_WHITE);
//   display.setCursor(0, 0);
//   display.println("TOAST Mk.I");
//   display.println("");
//   display.print("TS:");
//   display.println(timestamp);
//   display.println("");
//   display.print("VA_1:");
//   display.println(dio_status[0]);
//   display.print("VA_2:");
//   display.print(dio_status[1]);
//   display.print(" ");
//   display.println(dio_status[2]);
//   display.print("VA_3:");
//   display.print(dio_status[3]);
//   display.print(" ");
//   display.println(dio_status[4]);
//   display.print("VA_4:");
//   display.print(dio_status[5]);
//   display.print(" ");
//   display.println(dio_status[6]);
//   display.print("VA_5:");
//   display.print(dio_status[7]);
//   display.print(" ");
//   display.println(dio_status[8]);
//   display.print("VA_6:");
//   display.print(dio_status[9]);
//   display.print(" ");
//   display.println(dio_status[10]);
//   display.print("VA_7:");
//   display.println(dio_status[10]);
//   display.println("");
//   display.print("Vb_in:");
//   display.println(v_bat_int);
//   display.print("Vb_ex:");
//   display.println(v_bat_ext);
//   display.print("RSSI:");
//   display.println(LoRa_rx.packetRssi());
//   display.display();
// }

void serial_rx_handle(void){
  uint32_t i=0;
  memset(serial_rx_buf, 0, sizeof(serial_rx_buf));

  while(Serial.available()){
    serial_rx_buf[i++] = (uint8_t) Serial.read();
  }
  if(serial_rx_buf[0] != 0){
    digitalWrite(TX_LED_PIN, HIGH);
    for(uint8_t i = 0; i < LORA_PULSE_SIZE; i++){
      lora_send_command(serial_rx_buf[0]);
      delay(50);
    }
    digitalWrite(TX_LED_PIN, LOW);
  }
}

void seq_1(void){
  seq_1_active = 1;
  set_dio_pin(IGN_PIN);
  delay(IGNITER_ON_DELAY);
  clear_dio_pin(IGN_PIN);

  //V6A.write(SERVO_IGNITION);
  //V6B.write(SERVO_IGNITION);
  //dio_status[3] = 1;
  //dio_status[11] = 1;
  set_dio_pin(V5A_PIN);
  set_dio_pin(V5B_PIN);

  timer_1.trigger(SFT_IGN_DELAY);
  return;
}

void seq_2(void){
  seq_2_active = 1;
  V6A.write(SERVO_OPEN);
  //V6B.write(SERVO_OPEN);
  dio_status[3] = 1;
  //dio_status[11] = 1;

  timer_2.trigger(CF_OPEN_DELAY);
  return;
}

void seq_3(void){
  //seq_3_active = 1;
  return;
}

void timer1_callback(void){
  //set_dio_pin(V4A_PIN);
  set_dio_pin(V4B_PIN);
  V6A.write(SERVO_OPEN);
  clear_dio_pin(V5A_PIN);
  clear_dio_pin(V5B_PIN);
  //V6B.write(SERVO_OPEN);
  dio_status[2] = 1;
  //dio_status[3] = 0;
  //dio_status[10] = 1;
  //dio_status[11] = 0;
  timer_2.trigger(SFT_BURN_DELAY);
}
void timer2_callback(void){
  V6A.write(SERVO_CLOSE);
  V6B.write(SERVO_CLOSE);
  dio_status[2] = 0;
  dio_status[10] = 0;
  dio_status[3] = 0;
  dio_status[11] = 0;
  //clear_dio_pin(V4A_PIN);
  //clear_dio_pin(V4B_PIN);
  set_dio_pin(D2B_PIN);
  set_dio_pin(D2A_PIN);
  seq_1_active = 0;
  seq_2_active = 0;
}
void timer3_callback(void){

}
void timer4_callback(void){

}
void timer5_callback(void){

}
void timer6_callback(void){

}
void timer7_callback(void){

}
void timer8_callback(void){

}

void set_dio_pin(uint8_t pin){
  digitalWrite(pin, HIGH);
  switch(pin){
    case DIO_0_PIN:
      dio_status[0] = 1;
      break;
    case DIO_1_PIN:
      dio_status[1] = 1;
      break;
    case DIO_2_PIN:
      dio_status[2] = 1;
      break;
    case DIO_3_PIN:
      dio_status[3] = 1;
      break;
    case DIO_4_PIN:
      dio_status[4] = 1;
      break;
    case DIO_5_PIN:
      dio_status[5] = 1;
      break;
    case DIO_6_PIN:
      dio_status[6] = 1;
      break;
    case DIO_7_PIN:
      dio_status[7] = 1;
      break;
    case DIO_8_PIN:
      dio_status[8] = 1;
      break;
    case DIO_9_PIN:
      dio_status[9] = 1;
      break;
    case DIO_10_PIN:
      dio_status[10] = 1;
      break;
    case DIO_11_PIN:
      dio_status[11] = 1;
      break;
    case DIO_12_PIN:
      dio_status[12] = 1;
      break;
    case DIO_13_PIN:
      dio_status[13] = 1;
      break;
    case DIO_14_PIN:
      dio_status[14] = 1;
      break;
    case DIO_15_PIN:
      dio_status[15] = 1;
      break;
    default:
      return;
  }
}

void clear_dio_pin(uint8_t pin){
  digitalWrite(pin, LOW);
  switch(pin){
    case DIO_0_PIN:
      dio_status[0] = 0;
      break;
    case DIO_1_PIN:
      dio_status[1] = 0;
      break;
    case DIO_2_PIN:
      dio_status[2] = 0;
      break;
    case DIO_3_PIN:
      dio_status[3] = 0;
      break;
    case DIO_4_PIN:
      dio_status[4] = 0;
      break;
    case DIO_5_PIN:
      dio_status[5] = 0;
      break;
    case DIO_6_PIN:
      dio_status[6] = 0;
      break;
    case DIO_7_PIN:
      dio_status[7] = 0;
      break;
    case DIO_8_PIN:
      dio_status[8] = 0;
      break;
    case DIO_9_PIN:
      dio_status[9] = 0;
      break;
    case DIO_10_PIN:
      dio_status[10] = 0;
      break;
    case DIO_11_PIN:
      dio_status[11] = 0;
      break;
    case DIO_12_PIN:
      dio_status[12] = 0;
      break;
    case DIO_13_PIN:
      dio_status[13] = 0;
      break;
    case DIO_14_PIN:
      dio_status[14] = 0;
      break;
    case DIO_15_PIN:
      dio_status[15] = 0;
      break;
    default:
      return;
  }
}