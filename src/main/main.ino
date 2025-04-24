#include <stdio.h>
#include "esp_log.h"
#include "config.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "mcpwm_setup.h"
#include <PID_v1.h>

#include <SPI.h>
#include "AD7190.h"



AD7190* ad7190 = NULL;

uint32_t rawAd7190Data = 0;
double rawAd7190Data_Channel0 = 0;   //current
double rawAd7190Data_Channel1 = 0;   //voltage
uint32_t AD7190_rawZero = 0x800000;  // Arbitrary number based on my setup
double t = 0;

uint32_t serial_setpoint = 0;
bool new_setpoint = false;

bool timeout = false;

bool initAd7190() {

  // Initialise SPI Port HSPI with default pins:
  // SCLK = 14, CIPO = 12, COPI = 13, SS = 15

  SPIClass* spi = new SPIClass(HSPI);


  uint8_t hspi_mosi_pin = 27;
  ad7190 = new AD7190(spi, hspi_mosi_pin, "A");  // A stands for Channel A

  if (ad7190->begin()) {
#ifdef MAIN_DEBUG_VERBOSE
    Serial.println(F("AD7190 begin: OK"));
    Serial.print("Device name: ");
    Serial.println(ad7190->getDeviceName());
#endif

    float t = ad7190->getTemperature();
    Serial.print(F("AD7190 Temperature: "));
    Serial.println(t);

    return true;

#ifdef MAIN_DEBUG_VERBOSE
  } else {

    Serial.println(F("AD7190 begin: FAIL"));
#endif
  }
  return false;
}

void configureAd7190ContinuousRead() {

  // Set GPIOS required for development board:
  //AD7190_GPOCON_GP32EN
  uint8_t regGPIOSettings = (0 << 5) | AD7190_GPOCON_BPDSW;  //  Set excitation source  (P3 to low is 5V exitation on)
  ad7190->setRegisterValue(AD7190_REG_GPOCON, regGPIOSettings, 1, AD7190_CS_CHANGE);

  // Set AD7190 configuration
  uint32_t regConfigSettings = ((CHOP << 23) | AD7190_CONF_CHAN(AD7190_CH_AIN1P_AIN2M | AD7190_CH_AIN3P_AIN4M) | (REFDET << 6) | (BUF << 4) | (POL << 3) | AD7190_CONF_GAIN(AD7190_CONF_GAIN_1));

  ad7190->setRegisterValue(AD7190_REG_CONF, regConfigSettings, 3, AD7190_CS_CHANGE);

  // Set AD7190 mode
  uint32_t regModeSettings = (AD7190_MODE_SEL(AD7190_MODE_CONT) | (DTA_STA << 20) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(AD7190_FILTER_RATE_80));

  ad7190->setRegisterValue(AD7190_REG_MODE, regModeSettings, 3, AD7190_CS_CHANGE);

  // Set Continuous Read Mode:
  uint8_t regCommSetting = (AD7190_COMM_READ | AD7190_COMM_ADDR(AD7190_REG_DATA) | AD7190_COMM_CREAD);

  ad7190->setModeContinuousRead(regCommSetting);

  Serial.println("configureAd7190ContinuousRead DONE");
}

int32_t getAd7190SampleContinuousRead() {


  timeout = !ad7190->waitRdyGoLow();

  rawAd7190Data = ad7190->getDataContinuousRead(4);
  t = rawAd7190Data >> 8;  // - AD7190_rawZero;
  if ((rawAd7190Data & 0xFF) != 0 & (rawAd7190Data & 0xFF) != 1)
    return 0;

  if (rawAd7190Data == 0)
    return 0;

  t = (double)(t) / (double)(3355.443);

  if ((rawAd7190Data & 0x07) == 1)
    rawAd7190Data_Channel1 = t * 2;
  else if ((rawAd7190Data & 0x07) == 0)
    rawAd7190Data_Channel0 = t;

#ifdef MAIN_DEBUG_CONVERSION

  if ((rawAd7190Data & 0x07) == 1) {
    Serial.print("CH1: ");
    Serial.print(2 * t);  // Print Raw ADC value in mV (mili-volt)
  }

  else if ((rawAd7190Data & 0x07) == 0) {
    Serial.print("CH0: ");
    Serial.print(t);  // Print Raw ADC value in mV (mili-volt)
  }

  Serial.print(", RAW: ");
  Serial.println(rawAd7190Data);

#endif

  return t;
}


//Define Variables we'll be connecting to
double Setpoint, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 0.001, aggKi = 0.025, aggKd = 0;
double consKp = 0.002, consKi = 0.0005, consKd = 0;

//Specify the links and initial tuning parameters
PID myPID(&rawAd7190Data_Channel1, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);




void read_serial(void *arg) {
  String teststr;
  new_setpoint = false;
  serial_setpoint = 0;

  while (true) {

    new_setpoint = false;

    while (Serial.available() == 0) {}     //wait for data available

    new_setpoint = true;
    serial_setpoint = Serial.parseInt(SKIP_ALL); //dataIn now holds 0
  }
}



void setup(void) {

  pinMode(4, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(5, OUTPUT);

  digitalWrite(4, LOW);
  digitalWrite(16, LOW);
  digitalWrite(16, LOW);
  digitalWrite(5, LOW);


  delay(3000);

  Serial.begin(115200);

  Setpoint = 2000.0;


  setup_mcpwm();
  Serial.println("\r\n MCPWM SETUP CONPLETE \r\n\r\n");

  pinMode(27, INPUT);
  initAd7190();                     //  Initialize AD7190
  configureAd7190ContinuousRead();  //  Set AD7190 to Continuous Read Mode


  myPID.SetOutputLimits(0.0, 179.5);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  mcpwm_force_lvl(false); //enable MOSFET signals
  update_phase(0);


  xTaskCreatePinnedToCore(read_serial, "read_serial_number", 2048, NULL, 2, NULL, 1);  // Pin to core 1 so PID performs seamlessly
}

void loop() {


  while (true) {

    getAd7190SampleContinuousRead();


    double gap = abs(Setpoint - rawAd7190Data_Channel1);  //distance away from setpoint
    if (gap < 0) {
      //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    } else {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }

    myPID.Compute();

    if(new_setpoint){
      Serial.print("\r\n\r\n   NEW SETPOINT\r\n\r\n");
      Setpoint = (double)serial_setpoint;
      new_setpoint = false;
    }

    Serial.print(Setpoint);
    Serial.print(", ");
    Serial.print(rawAd7190Data_Channel1); //Output voltage
    Serial.print(", ");
    Serial.print(rawAd7190Data_Channel0 * 125); //Output current
    Serial.print(", ");
    Serial.println(Output); //Phase difference

    update_phase(Output);

    delay(100);
  }
}
