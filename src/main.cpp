#include <Arduino.h>
#include "ICM_20948.h"
#define FREEIMU_SERIAL Serial
#define IMU_SPI_PORT spi_imu
#define IMU_SPI_CS_PIN PB12
#define IMU_SPI_SCLK_PIN PB13
#define IMU_SPI_MISO_PIN PB14
#define IMU_SPI_MOSI_PIN PB15

SPIClass spi_imu(IMU_SPI_MOSI_PIN,IMU_SPI_MISO_PIN,IMU_SPI_SCLK_PIN);
ICM_20948_SPI imu;

void mpuInit();
void mpuUpdate();
void freeIMUCalibration();
char serial_busy_wait();
void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes);
void writeVar(void * val, uint8_t type_bytes);

void setup()
{
  FREEIMU_SERIAL.begin(115200);
  mpuInit();
}

void loop()
{
  freeIMUCalibration();
}



void mpuInit()
{
  IMU_SPI_PORT.begin();
  do{
    // FREEIMU_SERIAL.println("IMU Initialating...");
    imu.begin( IMU_SPI_CS_PIN, IMU_SPI_PORT);//use default config 2G/250DPS
  }while(imu.status != ICM_20948_Stat_Ok);
  // FREEIMU_SERIAL.println("IMU Initialate Finish!");
}

// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void * val, uint8_t type_bytes) {
  byte * addr=(byte *)(val);
  for(uint8_t i=0; i<type_bytes; i++) { 
    FREEIMU_SERIAL.write(addr[i]);
  }
}

void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


char serial_busy_wait() {
  while(!FREEIMU_SERIAL.available()) {
    ; // do nothing until ready
  }
  return FREEIMU_SERIAL.read();
}


void freeIMUCalibration()
{
  int16_t raw_values[11];
  if(FREEIMU_SERIAL.available()) 
  {
    char cmd = FREEIMU_SERIAL.read();
    if(cmd=='v') {
      //sprintf(str, "FreeIMU library by FREQ: LIB_VERSION: %s", FREEIMU_LIB_VERSION);
      FREEIMU_SERIAL.print("OK....");
      FREEIMU_SERIAL.print('\n');
    }
    else if(cmd=='b') 
    {
      uint8_t count = serial_busy_wait();
      for(uint8_t i=0; i<count;) 
      {
        if(imu.dataReady())
        {
          imu.getAGMT();
          raw_values[0] = imu.agmt.acc.axes.x;
          raw_values[1] = imu.agmt.acc.axes.y;
          raw_values[2] = imu.agmt.acc.axes.z;

          raw_values[3] = imu.agmt.gyr.axes.x;
          raw_values[4] = imu.agmt.gyr.axes.y;
          raw_values[5] = imu.agmt.gyr.axes.z;

          raw_values[6] = imu.agmt.mag.axes.x;
          raw_values[7] = imu.agmt.mag.axes.y;
          raw_values[8] = imu.agmt.mag.axes.z;
          writeArr(raw_values, 9, sizeof(int16_t));
          FREEIMU_SERIAL.println();
          i++;
        } 
      }
    }
  }
}

