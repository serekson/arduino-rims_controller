#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mrf24j40_driver.h>
#include <mrf24j40_client.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <rims_driver.h>
#include <avr/wdt.h>

#define PIN_CLK 13    //SPI Clock            DI
#define PIN_SDI 12    //SPI MISO             DI
#define PIN_SDO 11    //SPI MOSI             DO
#define PIN_CS 10     //SPI SS                DI
#define PIN_INT 3     //MRF24J40 Interrupt    DO

#define PIN_TEMP A3   //Temp Sensor Pin
#define PIN_ELEMENT 8 //Element Enable Pin
#define PIN_PUMP 9    //Pump Relay enable pin
#define LCD_WIDTH 20
#define LCD_HEIGHT 4

MRFClient mrf(PIN_CS, PIN_INT);  //Create wireless object
RimsClass rims(PIN_TEMP, PIN_ELEMENT, PIN_PUMP);

uint8_t join_stat;
uint32_t log_int;
uint32_t log_time;

uint8_t pump_stat;
uint8_t rims_stat;

bool WD_en;

void setup() {
  Serial.begin(19200);
  
  initialize_mrf();
  rims.init();
  
  log_int = 10000;
  
  WD_en=1;
  wdt_enable(WDTO_8S);
}

void initialize_mrf() {
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  mrf.reset();
  mrf.init_client();

  attachInterrupt(1, mrf_isr, FALLING);   
  interrupts();
}

void mrf_isr() {
  mrf.int_mrf=1;
}

void proc_cmd(void) {
  int i;
  int cb_idx;
  double mash_temp;
  double kp;
  double ki;
  double kd;
  uint32_t log_id;
  
  byte rims_cmd;
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;

  cb_idx=0;
  rims_cmd=mrf.cmd_buffer[cb_idx++];

  switch(rims_cmd) {
    case 0x00:  //server update
      Serial.println("server update");
      tx_settings_update();
      break;
    case 0x01:  //pump enable
      rims.enablePump();
      pump_stat=1;
      Serial.println("pump enable");
      tx_status_update();
      break;
    case 0x02:  //pump disable
      if(rims_stat==0) {
        rims.disablePump();
        Serial.println("pump disable");
        pump_stat=0;
      }
      tx_status_update();
      break;
    case 0x03:  //rims_enable
      if(rims._element_status==0) {
        rims.enableElement();
      }
      tx_status_update();
      break;
    case 0x04:  //rims disable
      rims.disableElement();
      tx_status_update();
      break;
    case 0x05:  //update settings
      Serial.println("settings update: ");

      for(i=0;i<4;i++) {  //rx mash temp
        byte2var.asBytes[i] = mrf.cmd_buffer[cb_idx++];
      }
      mash_temp=byte2var.asDouble;
	  
      for(i=0;i<4;i++) {  //rx mash Kp
        byte2var.asBytes[i] = mrf.cmd_buffer[cb_idx++];
      }
      kp=byte2var.asDouble;
//      Serial.print("Kp: ");
//      Serial.println(kp);
	  
      for(i=0;i<4;i++) {  //rx mash Ki
        byte2var.asBytes[i] = mrf.cmd_buffer[cb_idx++];
      }
      ki=byte2var.asDouble;
//      Serial.print("Ki: ");
//      Serial.println(ki);
	  
      for(i=0;i<4;i++) {  //rx mash Kd
        byte2var.asBytes[i] = mrf.cmd_buffer[cb_idx++];
      }
      kd=byte2var.asDouble;
//      Serial.print("Kd: ");
//      Serial.println(kd);
      
      for(i=0;i<4;i++) {  //rx manual duty cycle
        byte2var.asBytes[i] = mrf.cmd_buffer[cb_idx++];
      }
      if(rims._man_duty < 0 || rims._man_duty > 1) {
        Serial.println("Man Duty Invalid");
      } else {
        rims._man_duty=byte2var.asDouble;
      }
      
      if(mash_temp < 0 || mash_temp > 200) {
        Serial.println("Mash Temp Invalid");
      } else {
        rims.setMashTemp(mash_temp);
      }
      
      if(kp < 0 || kp > 10000) {
        Serial.println("Kp Invalid");
      } else {
        if(ki < 0 || ki > 10000) {
          Serial.println("Ki Invalid");
        } else {
          if(kd < 0 || kd > 10000) {
            Serial.println("Kd Invalid");
          } else {
            rims.setPIDTuning(kp,ki,kd);
          }
        }
      }
      
      tx_settings_update();
      break;
    case 0x06:  //log enable
      Serial.println("log enabled");
      
      log_id = mrf.cmd_buffer[cb_idx++];
      rims.setLogID(log_id);
      log_int = 10000;
      log_time=millis()-5000;
      
      tx_status_update();
      break;
    case 0x07:  //log disable
      rims.setLogID(0);
      Serial.println("log disabled");
      tx_status_update();
      break;
    case 0x08:  //rims status
      tx_status_update();
      break;
    case 0x09:  //set rims auto
      Serial.println("RIMs Mode: Auto");
      rims._rims_mode=1;
      if(rims._element_status == 1) {
        rims.enableElement();
      }
      
      tx_status_update();
      break;
    case 0x0A:  //set rims manual
      Serial.println("RIMs Mode: Man");
      rims._rims_mode=0;
      tx_status_update();
      break;
    case 0xFF:
      Serial.println("Reset uC");
      WD_en=0;
      break;
    default:
      Serial.print("RIMs Command: ");
      Serial.println(rims_cmd);
  }
}

void tx_status_update() {
  int i;
  int tcb_idx;
  
  Serial.println("status update");
 
  tcb_idx=0;
  mrf.tx_cmd_buffer[tcb_idx++] = 101;
  mrf.tx_cmd_buffer[tcb_idx++] = rims.getLogID();
  mrf.tx_cmd_buffer[tcb_idx++] = rims._pump_status;
  mrf.tx_cmd_buffer[tcb_idx++] = rims._element_status;
  mrf.tx_cmd_buffer[tcb_idx++] = rims._rims_mode;
  
  mrf.tx_data_queue(tcb_idx);
}

void tx_settings_update() {
  int i;
  int tcb_idx;
  double mash_temp;
  double kp;
  double ki;
  double kd;  
  
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;
  
  Serial.println("server update");
  
  rims.getRimsStatus(&mash_temp, &kp, &ki, &kd);
  
  tcb_idx=0;
  mrf.tx_cmd_buffer[tcb_idx++] = 100;  //server update command
  
  byte2var.asDouble = mash_temp;
  for(i=0;i<4;i++) {  //rx mash temp
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  byte2var.asDouble = kp;
  for(i=0;i<4;i++) {  //rx kp
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  byte2var.asDouble = ki;
  for(i=0;i<4;i++) {  //rx ki
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  byte2var.asDouble = kd;
  for(i=0;i<4;i++) {  //rx kd
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  byte2var.asDouble = rims._man_duty;
  for(i=0;i<4;i++) {
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  mrf.tx_data_queue(tcb_idx);
}

void tx_temp() {
  uint8_t i;
  uint8_t tcb_idx;
  
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;
  
  tcb_idx=0;
  
  mrf.tx_cmd_buffer[tcb_idx++] = 102;  //temp command
  
  byte2var.asUINT32 = rims.getLogID();
  for(i=0;i<4;i++) {  //tx log id
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
    //Serial.println(mrf.tx_cmd_buffer[tcb_idx-1]);
  }
  
  byte2var.asDouble = rims.getTemp();
  for(i=0;i<4;i++) {  //tx temp
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
    //Serial.print(byte2var.asBytes[i], HEX);
    //Serial.print(" ");
  }
  //Serial.println("");
  Serial.print("log temp: ");
  Serial.println(byte2var.asDouble);
  
  byte2var.asDouble = rims.getSetTemp();
  for(i=0;i<4;i++) {  //tx temp
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  byte2var.asDouble = rims.getDutyCycle();
  for(i=0;i<4;i++) {  //tx duty cycle
    mrf.tx_cmd_buffer[tcb_idx++] = byte2var.asBytes[i];
  }
  
  mrf.tx_cmd_buffer[tcb_idx++] = rims._pump_status;
  mrf.tx_cmd_buffer[tcb_idx++] = rims._element_status;

  mrf.tx_data_queue(tcb_idx);
}

void loop() {
  rims.run();
  mrf.client_loop();

  if(mrf.cmd_count>0) {
    proc_cmd();
    //Serial.println("Proc Command");
    mrf.cmd_count--;
  }

  if(join_stat!=mrf._join_stat) {
    rims.updateMRFstat(mrf._join_stat);
    
    if(mrf._join_stat == 4 && join_stat == 3) {
      tx_settings_update();
      tx_status_update();
    }
    
    join_stat = mrf._join_stat;
  }

  if(WD_en) {
    wdt_reset();
  }
}
