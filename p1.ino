/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *PS: Sensor codes were added and everything was adapted for the LoRa 32 board
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = {0x9D,0x83,0x33,0x32,0x50,0xBF,0x51,0x1B,0x59, 0x46, 0xB7, 0xEE, 0xDC, 0xC8, 0x71, 0xDB};

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = {0x13, 0xDD, 0x39, 0xAD, 0xC8, 0xD3, 0xE7, 0x7A, 0x4C, 0x56, 0x6F, 0xDE, 0x76, 0x22, 0xAB, 0x4F};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x260136B1; // <-- Change this address for every node!

// Pinos do display (comunicação i2c)
  const int DISPLAY_ADDRESS_PIN = 0x3c;
  const int DISPLAY_SDA_PIN = 4;
  const int DISPLAY_SCL_PIN = 15;
  const int DISPLAY_RST_PIN = 16;

// Pinos do lora (comunicação spi)
  const int LORA_SCK_PIN = 5;
  const int LORA_MISO_PIN = 19;
  const int LORA_MOSI_PIN = 27;
  const int LORA_SS_PIN = 18;
  const int LORA_RST_PIN = 14;
  const int LORA_DI00_PIN = 26;

// Frequência de comunicação
  const int BAND = 868E6;

// Contador de pacotes enviados via lora
  int counter = 0;
  
// Altura da fonte (correspondente a fonte ArialMT_Plain_16)
  const int fontHeight = 16; 
    
// Objeto do display
  SSD1306 display(DISPLAY_ADDRESS_PIN, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);

//Pino do Sensor de temperatura
  const int DS18B20 = 23;

// Pino do Sensor de turbidez
  const int SensorTurbidez = 13;

// Pino do Sensor de nivel
  const int Sensornivel = 2;

// Pino do sensor de PH
  const int SensorPh = 32; 

// Iniciamos as variáveis
  int i;
  float temp;
  float volt;
  float NTU;
  uint8_t value;
  String nivel;
  
//Configuramos uma instancia OneWire para se comunicar com o sensor
  OneWire ourWire(DS18B20);

//A biblioteca DallasTemperature utiliza a OneWire
  DallasTemperature sensors(&ourWire);

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Pin mapping for the SparkX ESP32 LoRa 1-CH Gateway
const lmic_pinmap lmic_pins = {
  .nss = LORA_SS_PIN ,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST_PIN ,
  .dio = {LORA_DI00_PIN, 35, 34},
};



// Função que inicializa o display
bool displayBegin()
{
  // Reiniciamos o display
  pinMode(DISPLAY_RST_PIN, OUTPUT);
  digitalWrite(DISPLAY_RST_PIN, LOW);
  delay(1);
  digitalWrite(DISPLAY_RST_PIN, HIGH);
  delay(1);
  return display.init(); 
}

// Função que faz algumas configuções no display
void displayConfig()
{
  // Invertemos o display verticalmente
  display.flipScreenVertically();
  // Setamos a fonte
  display.setFont(ArialMT_Plain_16);
  // Alinhamos a fonta à esquerda
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

// Função que arredonda o valor
  float ArredondarPara( float ValorEntrada, int CasaDecimal ) {
    float multiplicador = powf( 10.0f, CasaDecimal );
    ValorEntrada = roundf( ValorEntrada * multiplicador ) / multiplicador;
  return ValorEntrada;
  }



// Carga útil para enviar ao gateway TTN
static uint8_t payload[10];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
            */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.println(LMIC.dataLen);
                 Serial.println(F(" bytes of payload"));
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {                    

              //Leitura do sensor de temperatura
              //Solicitamos a temperatura do sensor
               sensors.requestTemperatures();
               temp = sensors.getTempCByIndex(0);
               float tempmeasure = temp / 1000;
               
     
              //Leitura do sensor de turbidez
              // Realiza a soma dos "i" valores da tensao
              for (i = 0; i < 800; i++) {
                  volt += ((float)analogRead(SensorTurbidez) / 1023) * 5;
              }
              // Realiza a média entre os valores lidos na função for acima
               volt = volt / 800;
               volt = ArredondarPara(volt, 1); 
             // Se Volt menor que 2.5 fixa o valor de NTU
                if (volt < 2.5) {
                     NTU = 3000;
                } 
                else if (volt > 4.2) {
                     NTU = 0;
                     volt = 4.2;
                } 
             // Senão calcula o valor de NTU através da fórmula
                 else {
                      NTU = -1120.4 * pow(volt,2.0) + 5742.3 * volt - 4353.8;
                 }
                 
                 float NTUmeasure = NTU / 10000;
                 
    
             //Leitura do sensor de nivel
                 value = digitalRead(Sensornivel);
                
                 

             //Leitura do sensor de PH
                 int measure = analogRead(SensorPh);
                 float voltPh = (5 / 6000.0) * measure;
                 float Ph = 14 - ((voltPh - 0.06) * 2.8);
                 float Phmeasure  = Ph / 100;


                 // float -> int
                 // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
                     uint16_t payloadTemp = LMIC_f2sflt16(tempmeasure);
                 // int -> bytes
                     byte tempLow = lowByte(payloadTemp);
                     byte tempHigh = highByte(payloadTemp);
                 // place the bytes into the payload
                     payload[0] = tempLow;
                      payload[1] = tempHigh;
 
                 // float -> int
                      uint16_t payloadNTU = LMIC_f2sflt16(NTUmeasure);
                 // int -> bytes
                      byte NTULow = lowByte(payloadNTU);
                      byte NTUHigh = highByte(payloadNTU);
                      payload[2] = NTULow;
                      payload[3] = NTUHigh;

                 // float -> int
                      uint16_t payloadvalue = LMIC_f2sflt16(value);
                 // int -> bytes
                      byte ValueLow = lowByte(payloadvalue);
                      byte ValueHigh = highByte(payloadvalue);
                      payload[4] = ValueLow;
                      payload[5] = ValueHigh;

                 // float -> int
                      uint16_t payloadPh = LMIC_f2sflt16(Phmeasure);
                 // int -> bytes
                      byte PhLow = lowByte(payloadPh);
                      byte PhHigh = highByte(payloadPh);
                      payload[6] = PhLow;
                      payload[7] = PhHigh;
 

             //Serial

                  Serial.print("Leitura: "); 
                  Serial.println(counter+1);
    
             //Sensor de Temperatura
                   Serial.print("Temperatura: "); 
                   Serial.print(sensors.getTempCByIndex(0)); 
                   Serial.println(" *C");
      
            //Sensor de Turbidez
                   Serial.print("Tensao: ");
                   Serial.print(volt);
                   Serial.print("\tNTU: ");
                   Serial.println(NTU);
   
            // Sensor de Nivel
                   Serial.print("Sensor de nivel: ");
                   Serial.println(value);
    
            //Sensor de PH  
                   Serial.print("Measure: ");
                   Serial.print(measure);
                   Serial.print("\tTensao: ");
                   Serial.print(voltPh, 2);
                   Serial.print(" V: ");
                   Serial.print("\tPH: ");
                   Serial.println(Ph, 1);
  

      
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);
        // Incrementamos o contador
          counter++;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

    //Setamos o pino dos sensores como entrada
       pinMode(DS18B20, INPUT);
       pinMode(SensorTurbidez, INPUT);
       pinMode(Sensornivel, INPUT);
       pinMode(SensorPh, INPUT);    
       
    // Iniciamos a comunicação SPI
       SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SS_PIN);
       Serial.begin(9600);
       delay(100);
       Serial.println(F("Starting"));

    // Inciamos o sensor de temperatura
  sensors.begin();
  // Esperamos 1s
  delay(1000);
    
    // Iniciamos o display
  if(!displayBegin())
  {
    // Se não deu certo, exibimos falha de display na serial
    Serial.println("Display failed!");
    // E deixamos em loop infinito
    while(1);
  }
  // Configuramos o posicionamento da tela, fonte e o alinhamento do texto
  displayConfig();
  

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
   
   
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    LMIC_setupChannel ( 0 , 868100000 , DR_RANGE_MAP (DR_SF12, DR_SF7), BAND_CENTI);      // banda g
    #endif
    
    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
        LMIC.dn2Dr = DR_SF9;
 
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
Serial.println("Freq");
Serial.println(LMIC.freq);
}



void loop() 
{
    // Variável usada para indicar em qual linha o cursor deverá estar quando uma mensagem no display for exibida
  int line;
  // Inicia a leitura da tensao em 0
  volt = 0;

 // Limpamos o display
  display.clear();
  
  
//Display 

  // Iniciamos na primeira linha (zero)
  line = 0;

  
//    Escrevemos a mensagem "Sending: "
   display.drawString(0, line, "Sending: ");
   line++;
   display.drawString(0, line * fontHeight, String(counter));
   // Exibimos as alterações no display
 display.display();

  // Enviamos um pacote 
//  LoRa.beginPacket();
//  LoRa.print("Temperatura: ");
//  LoRa.print(sensors.getTempCByIndex(0));
//  LoRa.print("NTU: ");
//  LoRa.print(NTU);
//  LoRa.print("Nivel: ");
//  LoRa.print(value);  
//  LoRa.print("\tPH: ");
//  LoRa.print(Ph, 1);
//  LoRa.print(counter);
//  LoRa.endPacket();
 
 // Aguardamos 1` segundos
  delay(1000);

    
    os_runloop_once();
}
