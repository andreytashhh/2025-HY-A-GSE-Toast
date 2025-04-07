#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include <LoRa.h>
//#include <TeensyTimerTool.h>

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

#define UART_RX_PIN       (46)//(34)
#define UART_TX_PIN       (47)//(35)


// Lora Defines
#define LORA_TX_FREQ    (870E6)
#define LORA_RX_FREQ    (866E6)

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
#define valve_nb          (3) //nombre de valves total
#define SERIAL_RX_BUF_SIZE (5)
//#define DIO_QTY           (16)
//#define AN_QTY            (8)

// LoRa Commands
#define LORA_CMD_ABORT          (0xBB)

#define LORA_CMD_VQ_D_ON        (0x60)
#define LORA_CMD_VQ_D_OFF       (0x61)
#define LORA_CMD_VFILL_ON       (0x90)
#define LORA_CMD_VFILL_OFF      (0x91)
#define LORA_CMD_VPURGE_ON      (0x20)
#define LORA_CMD_VPURGE_OFF     (0x21)

// Pin Distribution
#define V_QUICK_DISC_PIN   (DIO_2_PIN)
#define V_FILL_PIN         (DIO_3_PIN)
#define V_PURGE_PIN        (DIO_4_PIN)

//taille paquets de données
#define pack_uart_size            (96) //taille paquet de communication Arduino-Teensy
#define pack_uart_size_oct        (12) //same, mais en octets
#define lora_tx_pack_size         (13) //taille du paquet en OCTETS a envoyer a l'operateur pour 99 bits (=etat des 3 valves = 3 bits, loadcells = 4*24 bits = 96)

// Function Definitions
void GPIO_init(void);
void lora_init(void);
void lora_tx_handler(void);
void lora_rx_handler(void); 
void lora_parse(uint8_t *buffer);
//void command_parse(uint8_t *buffer);
void command_parse(int cmd_nb);
void lora_packet_build(void);
void lora_send_command(uint8_t command);
//void analog_read(void);
void uart_read(void);
void set_dio_pin(uint8_t pin);
void clear_dio_pin(uint8_t pin);
void lora_packaging(void);

// LoRa Variables
LoRaClass LoRa_rx;
LoRaClass LoRa_tx;

uint8_t lora_rx_buf[LORA_PACKET_LEN];
uint8_t lora_tx_buf[LORA_PACKET_LEN];
int lora_rssi;

// Status Variables
//uint8_t dio_status[DIO_QTY] = {0};
//uint16_t analog_status[AN_QTY] = {0};
bool valve_status[valve_nb] = {0}; //les 3 premiers bits (0, 1, 2) 
//indiqueront l'etat des valves (1 = ouvert, 0 = ferme): 0 = QCK_DISC, 1 = FILL, 2 = PURGE

// Misc Variables
uint32_t timestamp;
uint8_t serial_rx_byte = 0;
uint8_t serial_rx_buf[SERIAL_RX_BUF_SIZE];
uint32_t lora_tx_counter = 0;


//other variables
uint8_t load_cells[pack_uart_size_oct]; //tableau contenant les valeurs des loads cells
uint8_t LoRa_package[lora_tx_pack_size]; //tableau contenant les valeurs a envoyer a l'operateur (etat des 3 valves, loadcells)

void setup() {
  // put your setup code here, to run once:
  GPIO_init();
  delay(1000);

  lora_init();
  //fermer tt les vannes
  command_parse(LORA_CMD_ABORT);
  //initialize serial communication (pin 46, 47)
  Serial5.begin(58824);  
}

void loop() {
  // put your main code here, to run repeatedly:
  //analog_read();

  uint32_t current_millis = millis();
  if((current_millis - lora_tx_counter) >= LORA_TX_PERIOD) {
    lora_tx_counter = current_millis;
    lora_packet_build();
    lora_tx_handler();
  }
  lora_rx_handler();
  //read UART
  if (Serial5.available()){
    uart_read();
  }

}

// put function definitions here:
void GPIO_init(){ //à racourcir
/*  pinMode(RX_LED_PIN, OUTPUT);
//  pinMode(TX_LED_PIN, OUTPUT);
//  pinMode(STATUS_LED_PIN, OUTPUT);
//  pinMode(DIO_0_PIN, OUTPUT);
pinMode(DIO_1_PIN, OUTPUT);*/
  pinMode(DIO_2_PIN, OUTPUT);
  pinMode(DIO_3_PIN, OUTPUT);
  pinMode(DIO_4_PIN, OUTPUT);
/*  pinMode(DIO_5_PIN, OUTPUT);
  pinMode(DIO_6_PIN, OUTPUT);
  pinMode(DIO_7_PIN, OUTPUT);
  pinMode(DIO_8_PIN, OUTPUT);
  pinMode(DIO_9_PIN, OUTPUT);
  pinMode(DIO_10_PIN, OUTPUT);
  pinMode(DIO_11_PIN, OUTPUT);
  pinMode(DIO_12_PIN, OUTPUT);
  pinMode(DIO_13_PIN, OUTPUT);
  pinMode(DIO_14_PIN, OUTPUT);
  pinMode(DIO_15_PIN, OUTPUT);
  pinMode(A_0_PIN, INPUT_PULLDOWN);
  pinMode(A_1_PIN, INPUT_PULLDOWN);*/
  /*
    pinMode(A_2_PIN, INPUT_PULLDOWN);
    pinMode(A_3_PIN, INPUT_PULLDOWN);
  */
  /*
  pinMode(A_4_PIN, INPUT_PULLDOWN);
  pinMode(A_5_PIN, INPUT_PULLDOWN);
  pinMode(A_6_PIN, INPUT_PULLDOWN);
  pinMode(A_7_PIN, INPUT_PULLDOWN);*/
  pinMode(UART_RX_PIN, INPUT_PULLDOWN);
  pinMode(UART_TX_PIN, OUTPUT);

/*  digitalWrite(RX_LED_PIN, LOW);
  digitalWrite(TX_LED_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(DIO_0_PIN, LOW);
  digitalWrite(DIO_1_PIN, LOW);*/
  digitalWrite(DIO_2_PIN, LOW);
  digitalWrite(DIO_3_PIN, LOW);
  digitalWrite(DIO_4_PIN, LOW);
/*  digitalWrite(DIO_5_PIN, LOW);
  digitalWrite(DIO_6_PIN, LOW);
  digitalWrite(DIO_7_PIN, LOW);
  digitalWrite(DIO_8_PIN, LOW);
  digitalWrite(DIO_9_PIN, LOW);
  digitalWrite(DIO_10_PIN, LOW);
  digitalWrite(DIO_11_PIN, LOW);
  digitalWrite(DIO_12_PIN, LOW);
  digitalWrite(DIO_13_PIN, LOW);
  digitalWrite(DIO_14_PIN, LOW);
  digitalWrite(DIO_15_PIN, LOW);*/
}
void lora_init(){
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
  //LoRa_rx.receive();
  //LoRa_rx.onReceive(lora_rx_handler);

  LoRa_tx.setPins(TX_LORA_CS_PIN, NO_RESET,TX_LORA_IRQ_PIN);
  LoRa_tx.setSPI(SPI);
  if (!LoRa_tx.begin(LORA_TX_FREQ)) {
    while (1);
  }
  LoRa_tx.setTxPower(LORA_TX_PWR);
  LoRa_tx.setSpreadingFactor(LORA_SF);
}
void lora_tx_handler(){
  // send pacpacketSizeket
  //Serial.println("TX Handling");
  digitalWrite(TX_LED_PIN, HIGH);
  LoRa_tx.beginPacket();
  LoRa_tx.write(TOAST_IDENTIFIER); // TOAST
  LoRa_tx.write(STATUS_PACKET); // Packet ID
  for(int i=0;i<LORA_PACKET_LEN;i++){
    LoRa_tx.write(lora_tx_buf[i]);
  }
  LoRa_tx.endPacket(true);
  lora_rssi = LoRa_tx.packetRssi();
  digitalWrite(TX_LED_PIN, LOW);
  //Serial.println("Packet sent via tx");
}

void lora_rx_handler(){
  // try to parse packet
  memset(lora_rx_buf, 0, sizeof(lora_rx_buf));
  int packetSize = LoRa_rx.parsePacket();
  if (packetSize) {
    // received a packet
    digitalWrite(RX_LED_PIN, HIGH);
    // Read packet
    for(int i = 0; i < packetSize; i++) {
      lora_rx_buf[i] = LoRa_rx.read();
    }
    // Verify if it is a TOAST packet
    if (lora_rx_buf[0] == TOAST_IDENTIFIER) {
      lora_parse(lora_rx_buf);
    }
    lora_rssi = LoRa_rx.packetRssi();
    digitalWrite(RX_LED_PIN, LOW);
  }
}

void lora_parse(uint8_t *buffer){
  if(buffer[PACKET_ID] == COMMAND_PACKET){
    tone(BUZZER_PIN, 3000, 100);
    delay(50);
    tone(BUZZER_PIN, 4000, 100);

    uint8_t valves_received = buffer[PACKET_PAYLOAD];

    // Traiter chaque valve en fonction des bits reçus
    if (valves_received & 0x01)
      command_parse(LORA_CMD_VQ_D_ON);
    else
      command_parse(LORA_CMD_VQ_D_OFF);

    if (valves_received & 0x02)
      command_parse(LORA_CMD_VFILL_ON);
    else
      command_parse(LORA_CMD_VFILL_OFF);

    if (valves_received & 0x04)
      command_parse(LORA_CMD_VPURGE_ON);
    else
      command_parse(LORA_CMD_VPURGE_OFF);
  }
  else if(buffer[PACKET_ID] == STATUS_PACKET){
    //status_parse(buffer); // actuel comportement, laissé tel quel
  }
  else{
    tone(BUZZER_PIN, 3000, 500);
    delay(500);
    tone(BUZZER_PIN, 3000, 500);
  }
}
/*void analog_read(void){
  analog_status[0] = analogRead(A_0_PIN);
  analog_status[1] = analogRead(A_1_PIN);
  analog_status[2] = analogRead(A_2_PIN);
  analog_status[3] = analogRead(A_3_PIN);
  analog_status[4] = analogRead(A_4_PIN);
  analog_status[5] = analogRead(A_5_PIN);
  analog_status[6] = analogRead(A_6_PIN);
  analog_status[7] = analogRead(A_7_PIN);
}*/
void uart_read(void){
  //lire (pack_uart_size) bits = 9 bytes
  //read the 9 bytes and put them into load_cells (global variable)
  for (int i = 0; i < pack_uart_size_oct; i++) {
    load_cells[i] = Serial5.read();
  }
}

void command_parse(int cmd_nb){
//void command_parse(uint8_t *buffer){
  //recoit le "numero" d'une commande a executer et l'execute
  //(p.ex. ouvrir/fermer valves et notifier leur statut) 
  //switch(buffer[PACKET_PAYLOAD]){
    // DIO Commands
  switch(cmd_nb){
    case LORA_CMD_VQ_D_ON:
      set_dio_pin(V_QUICK_DISC_PIN);
      valve_status[0] = 1;
      break;
    case LORA_CMD_VQ_D_OFF:
      clear_dio_pin(V_QUICK_DISC_PIN);
      valve_status[0] = 0;
      break;
    case LORA_CMD_VFILL_ON:
      set_dio_pin(V_FILL_PIN);
      valve_status[1] = 1;
      break;
    case LORA_CMD_VFILL_OFF:
      clear_dio_pin(V_FILL_PIN);
      valve_status[1] = 0;
      break;
    case LORA_CMD_VPURGE_ON:
      set_dio_pin(V_PURGE_PIN);
      valve_status[2] = 1;
      break;
    case LORA_CMD_VPURGE_OFF:
      clear_dio_pin(V_PURGE_PIN);
      valve_status[2] = 0;
      break;
    
    case LORA_CMD_ABORT: //Abort
      //close all valves
      //appelee dans SETUP!
      clear_dio_pin(V_QUICK_DISC_PIN);
      clear_dio_pin(V_FILL_PIN);
      clear_dio_pin(V_PURGE_PIN);
      valve_status[0] = 1;
      valve_status[1] = 1;
      valve_status[2] = 1;
      break;
    default:
      return;
  }
}
void lora_packet_build(void) {
  // On réinitialise le buffer de transmission
  memset(lora_tx_buf, 0, LORA_PACKET_LEN);

  // Construction du paquet selon la convention :
  // Octet 0 : états des 3 vannes (bits : 0 = Quick Disconnect, 1 = FILL, 2 = PURGE)
  // Octets 1 à 12 : données load cell reçues via UART (96 bits, soit 12 octets)
  uint8_t valveByte = 0;
  valveByte |= (valve_status[0] ? 1 : 0);   // Bit 0 : Quick Disconnect (DIO_2_PIN)
  valveByte |= (valve_status[1] ? 2 : 0);   // Bit 1 : Valve FILL (PWM sur DIO_3_PIN)
  valveByte |= (valve_status[2] ? 4 : 0);   // Bit 2 : Valve PURGE (PWM sur DIO_4_PIN)
  lora_tx_buf[0] = valveByte;

  // Copier les 12 octets de données UART (load_cells) dans le paquet
  for (int i = 0; i < pack_uart_size_oct; i++) {
    lora_tx_buf[i + 1] = load_cells[i];
  }
  // Les octets restants de lora_tx_buf sont laissés à 0
}

void lora_send_command(uint8_t command){
  
  //uint32_t i=0;
  digitalWrite(TX_LED_PIN, HIGH);
  //Serial.println(command);
  LoRa_tx.beginPacket();
  LoRa_tx.write(TOAST_IDENTIFIER); // TOAST
  LoRa_tx.write(COMMAND_PACKET); // Packet ID
  // while(serial_rx_buf[i] != 0){
  for(uint8_t i = 0; i < SERIAL_RX_BUF_SIZE; i++){ //#define SERIAL_RX_BUF_SIZE (5)
    LoRa_tx.write(serial_rx_buf[i]);
    //Serial.print(serial_rx_buf[i]);
  }
  LoRa_tx.endPacket(true);
  digitalWrite(TX_LED_PIN, LOW);
  
}

void set_dio_pin(uint8_t pin){
  //mettre un port en sortie a 1 et changer son statut (p.ex. dasn valve_status)
  digitalWrite(pin, HIGH);
  switch(pin){
    case V_QUICK_DISC_PIN:
      valve_status[0] = 1;
      break;
    case V_FILL_PIN:
      valve_status[1] = 1;
      break;
    case V_PURGE_PIN:
      valve_status[2] = 1;
      break;
    default:
      return;
  }
}
void clear_dio_pin(uint8_t pin){
  digitalWrite(pin, LOW);
  switch(pin){
    case V_QUICK_DISC_PIN:
      valve_status[0] = 0;
      break;
    case V_FILL_PIN:
      valve_status[1] = 0;
      break;
    case V_PURGE_PIN:
      valve_status[2] = 0;
      break;
    default:
      return;
  }
}
void lora_packaging(void){
  //modifie le paquet LoRa_package de (99 bits/)13 octets pour qu'il puisse etre envoye par une autre fct
  //valve_status
  uint8_t valv_stat(1*valve_status[0]+2*valve_status[1]+4*valve_status[0]);
  LoRa_package[0] = valv_stat;
  //loadcells
  for (size_t i(0); i < pack_uart_size_oct; i++){
    LoRa_package[i+1] = load_cells[i];
  }
}