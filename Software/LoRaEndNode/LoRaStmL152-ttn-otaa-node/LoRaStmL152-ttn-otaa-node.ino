/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network. It's pre-configured for the Adafruit
 * Feather M0 LoRa.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/
//
//
//
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <DS18B20.h>
#include <extEEPROM.h> 
#include "Adafruit_Si7021.h"
#include "Adafruit_MPL3115A2.h"
#include "Adafruit_LTR390.h"

//
// UART1 RX and TX pins
//
HardwareSerial Uart1(PB7, PB6);
//
// DS18B20 Temperature Probe
//
DS18B20 ds(PA1);
//
// I2C-2
//
TwoWire Wire2(PB11,PB10) ;
//
// 24LC32A
// I2C-1
extEEPROM eeProm(kbits_32, 1, 32, 0x50);         //device size, number of devices, page size, I2C bus address
//
// si7020 Temperature and humidity sensor
// I2C-2
//
Adafruit_Si7021 si7020 = Adafruit_Si7021(&Wire2);
//
//
//
Adafruit_MPL3115A2 mpl3115 = Adafruit_MPL3115A2();
//
//
//
Adafruit_LTR390 ltr390 = Adafruit_LTR390();
//
// Application EUI, Device EUI,  and Application key
// These are stored in the eePROM on the sensor I/O terminal board 24LC32A
//
uint8_t AppEUI[8] ;      // Application EUI, LSB First
uint8_t DevEUI[8] ;      // Device EUI, LSB First
uint8_t AppKey[16] ;     // Application Key, MSB First

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// ---------------------- This is from orginal code for Lora main board --------------------
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// Uart1.
//static const u1_t PROGMEM APPEUI[8]= { 0xD4, 0x1F, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 } ;

// APPEUI is now Join EUI
//static const u1_t PROGMEM JoinEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } ;
//void os_getArtEui (u1_t* buf) { memcpy_P(buf, JoinEUI, 8);}

// This should also be in little endian format, see above.
//static const u1_t PROGMEM DEVEUI[8]= { 0x46, 0x8E, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 } ;
//void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
//static const u1_t PROGMEM APPKEY[16] = { 0x76, 0x23, 0x76, 0x7C, 0xB7, 0x77, 0x12, 0x40, 0xC3, 0x62, 0xEA, 0xF9, 0xE3, 0xC8, 0x2A, 0xF1 } ;
//void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//
// AppEUI, DevEUI, and DevKey are read from eeProm
//
void os_getArtEui (u1_t* buf) { memcpy(buf, AppEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy(buf, DevEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy(buf, AppKey, 16);}
//
// LoRa Payload
//
//static uint8_t mydata[] = "Hello, world!";
static uint8_t payload[16] ;
static osjob_t sendjob;
//
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//
//const unsigned TX_INTERVAL = 60;  // 60 Seconds
const unsigned TX_INTERVAL = 6*60;  // 6 minutes

// Pin mapping
// nss : SPI CS
// rst : LoRa device reset
// dio : LoRa GIO pins 0, 1, 2 from LoRa device 0 = INT, 1 = LoRa mode, 2 = FSK mode
const lmic_pinmap lmic_pins = {
    .nss = PA15,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PA8,
    .dio = {PA11, PA12, LMIC_UNUSED_PIN},
    //.rxtx_rx_active = 0,
    //.rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    //.spi_freq = 8000000,
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Uart1.print('0');
    Uart1.print(v, HEX);
}
//
//
//
void onEvent (ev_t ev) {
    Uart1.print(os_getTime());
    Uart1.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Uart1.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Uart1.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Uart1.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Uart1.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Uart1.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Uart1.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Uart1.print("netid: ");
              Uart1.println(netid, DEC);
              Uart1.print("devaddr: ");
              Uart1.println(devaddr, HEX);
              Uart1.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Uart1.print("-");
                printHex2(artKey[i]);
              }
              Uart1.println("");
              Uart1.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Uart1.print("-");
                      printHex2(nwkKey[i]);
              }
              Uart1.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Uart1.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Uart1.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Uart1.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Uart1.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Uart1.println(F("Received ack"));
            if (LMIC.dataLen) {
              Uart1.println(F("Received "));
              Uart1.println(LMIC.dataLen);
              Uart1.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Uart1.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Uart1.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Uart1.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Uart1.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Uart1.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Uart1.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Uart1.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Uart1.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Uart1.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Uart1.print(F("Unknown event: "));
            Uart1.println((unsigned) ev);
            break;
    }
}
//
// 
//
uint16_t battV ;                // Battery voltage
int16_t  tC_probe = 0;          // Temp from DS1820B
int16_t  tC_si702x = 0 ;        // Temp from si7020
int16_t  Humidity_si702x = 0 ;  // Humidity from si7020
uint16_t BaroPressure = 0 ;     // Barometric pressure from mpl3115
uint32_t UVlevel = 0 ;          // UV Level from LTR390
uint32_t ALSlevel = 0 ;         // Ambient light level from LTR390

//
//
//
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Uart1.println(F("OP_TXRXPEND, not sending"));
    } else {

        getBatteryVoltage( &battV ) ;

        getTemperature( &tC_probe ) ;

        //getAltPressure( &BaroPressure ) ;

        //getUVLevel ( &UVlevel ) ; 
        
        //get_si7020TempHumidity(&tC_si702x, &Humidity_si702x) ;

        payload[0]  = battV >> 8 ;
        payload[1]  = battV ;
        payload[2]  = tC_probe >> 8 ;
        payload[3]  = tC_probe  ;
        payload[4]  = tC_si702x >> 8   ;
        payload[5]  = tC_si702x ;
        payload[6]  = Humidity_si702x >> 8   ;
        payload[7]  = Humidity_si702x ;
        payload[8]  = UVlevel >> 16   ;
        payload[9]  = UVlevel >> 8   ;
        payload[10] = UVlevel ;
        payload[11] = BaroPressure >> 8   ;
        payload[12] = BaroPressure ;
        payload[13] = 0 ;
        payload[14] = 0 ;
        payload[15] = 0 ;

        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Uart1.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
/**
 * setup() 
 */
void setup() {

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW) ;
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH) ;

    pinMode(PA0, INPUT);   // LoRa Sensor I/O board 

    //REMAP SPI:
    SPI.setMOSI(PB5); // using pin number PYn
    SPI.setMISO(PB4); // using pin name PY_n
    SPI.setSCLK(PB3); // using pin number PYn
    SPI.setSSEL(PA15); // using pin number PYn

    // UART1
    while (! Uart1) ;
    Uart1.begin(115200);

    //Wire2.setClock(100000) ;

    //
    // si7020 Temperaure and humidity sensor
    //
/*
    if ( !si7020.begin() ) {
      Uart1.println("Can not find si7020");
      while (1);
    }
*/
    //
    // MPL3115 Bararometric pressure sensor
    //
/*
    if (! mpl3115.begin(&Wire2)) {
      Uart1.println("Can not find MPL3115A2");
      while (1);
    }
*/

/*
    // 
    // LTR390 ALS/UV sensor
    //
    if (! ltr390.begin(&Wire2)) {
      Uart1.println("Can not find LTR390");
      while (1);
    }
    //setLTR390mode(LTR390_MODE_UVS, LTR390_GAIN_3) ;
    setLTR390mode(LTR390_MODE_UVS, LTR390_GAIN_18, LTR390_RESOLUTION_20BIT) ;
*/

    //
    Uart1.println(F("Starting LoRa Node"));

    // Device ID eeProm
    uint8_t eepStatus = eeProm.begin(eeProm.twiClock400kHz);   //go fast!
    if (eepStatus) {
      Uart1.print(F("extEEPROM.begin() failed, status = "));
      Uart1.println(eepStatus);
      while (1);
    }

    // Display current configuration
    Uart1.println() ;
    Uart1.println(" --------------------------- ") ;
    Uart1.println("| Read AppEUI/DevEUI/AppKEY | ") ;
    Uart1.println(" --------------------------- ") ;
    eeProm.read(0xFD0, &AppEUI[0], 8);
    eeProm.read(0xFE0, &DevEUI[0], 8);
    eeProm.read(0xFF0, &AppKey[0], 16);
    displayEUIandKey() ;
    Uart1.println(" --------------------------- ") ;
    Uart1.println() ;

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}
//
// Get battery voltage
// Read from ADC is 12 bits but returned value from analogRead is 10 bits (Arduino ADC number of bits) 
// A resistor divider of 30K/10K in series scales the input voltage by Vbat/4 
// 5.0V  => 1.25
// 5.045 => 1.26125 - Drop out voltage @ 0.1A
// 5.25V => 1.3125  - Drop out voltage @ 0.5A
// 6.0V  => 1.5
// 7.0V  => 1.875
// 10V   => 2.5 ( Maximum input voltage to the LP38693 LDO 5V regulator )
// Regulator will drop out when Vin = 5.045V @ 0.100A or 5.250V @ 0.500A
//   
// Returns vbat * 1000 ;
//
void getBatteryVoltage( uint16_t *vbat )
{
    uint16_t val ;
     val  = analogRead(PA_6);
    *vbat = val*(3.30/1024)*1000.0 ;
}
//
// DS18B20 Temperature sensor
//
// Returns temperature in degrees Celsius*100
//
void getTemperature( int16_t *temp )
{
    float ftC ;
    int16_t tC ;
    ftC = ds.getTempC() ;
    tC = ftC * 100 ;
    *temp = tC ;
} 
//
// SI7020 Temperature and humidity sensor
//
void get_si7020TempHumidity( int16_t *tempC, int16_t *Hum )
{
    float tC, H ;

    tC = si7020.readTemperature() ;
    H  = si7020.readHumidity() ;
    tC = round(tC * 10.0) ;
    H  = round(H*10.0) ;
    
    //Uart1.print(tC);
    //Uart1.println("\n") ;
    //Uart1.print(H);
    //Uart1.println("\n") ;
    
     
    *tempC = tC ;
    *Hum = H ;
    //Uart1.println(*tempC);
    //Uart1.println(*Hum);
}
/**
 * MPLA3115A2 Altitude and Barometric pressure sensor
 */
void getAltPressure( uint16_t *baro )
{
  float pascals ;
  uint16_t pressure ;

  // Barometric Pressure
  pascals = mpl3115.getPressure();
  pressure = (pascals/10.0) ;
  //Uart1.print(pascals/3377); Serial.println(" Inches (Hg)");
  Uart1.print(pascals/100.0); Uart1.println(" hPa ");

  //float altm = baro3115.getAltitude();
  //Uart1.print(altm); Serial.println(" meters");

  //float tempC = baro3115.getTemperature();
  //Uart1.print(tempC); Serial.println("*C");

  *baro = pressure ;
}
/**
 * LTR390 Light Sensor
*/
void getUVLevel ( uint32_t *uvlevel ) 
{
  if (ltr390.newDataAvailable()) {
      *uvlevel = ltr390.readUVS() ;
      Uart1.print("UV data: ");
      Uart1.println(*uvlevel);
  }
}
//
//
//
void setLTR390mode ( ltr390_mode_t mode, ltr390_gain_t gain, ltr390_resolution_t resolution ) 
{
  ltr390.setMode(mode);

  ltr390.setGain(gain);

  ltr390.setResolution(resolution);

  ltr390.setThresholds(100, 1000);
  ltr390.configInterrupt(true, mode);
}
//
// displayEUIandKey()
//
void displayEUIandKey(void)
{
    int n ;
    Uart1.print("AppEUI :  ") ;
    for (n=0; n<8 ; n++)  {
       printHex2(AppEUI[n]) ;
       if (n < 7)
          Uart1.print("-");
    }
    Uart1.println("");
    Uart1.print("DevEUI :  ") ;
    for (n=0; n<8 ; n++)  {
       printHex2(DevEUI[n]);
       if ( n < 7 )
           Uart1.print("-");
    }
    Uart1.println("");
    Uart1.print("AppKey :  ") ;
    for (n=0; n<16 ; n++)  {
       printHex2(AppKey[n]) ;
       if ( n < 15 )
          Uart1.print("-");
    }
    Uart1.println("");
}
// *************************************************************************
// Main Loop
// *************************************************************************
void loop() {
    os_runloop_once();
}
