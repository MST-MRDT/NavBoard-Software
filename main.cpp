#include "RoveBoard_TivaTM4C1294NCPDT.h"
#include "RoveComm.h"
#include <stdio.h>
#include "RoveAdafruit_GPS.h"

//list of all registers binary addresses;
#include "IMUreg.h"

const uint16_t GPS_FIX_QUALITY_DATA_ID = 1296;
const uint16_t GPS_LAT_LON_DATA_ID = 1297;
const uint16_t GPS_SPEED_DATA_ID = 1298;
const uint16_t GPS_ANGLE_DATA_ID = 1299;
const uint16_t GPS_ALTITUDE_DATA_ID = 1300;
const uint16_t GPS_SATELLITES_DATA_ID = 1301;

const uint16_t IMU_TEMP_DATA_ID = 1313;
const uint16_t IMU_ACCEL_DATA_ID = 1314;
const uint16_t IMU_GYRO_DATA_ID = 1315;
const uint16_t IMU_MAG_DATA_ID = 1316;

void loop();
uint64_t gps_lat_lon = 0;

Adafruit_GPS GPS;
RoveI2C_Handle i2cHandle = roveI2cInit(0, I2CSPEED_FULL, PB_2, PB_3);
void setup()
{
  //connect to roveComm
  roveComm_Begin(192,168,1,133);

  i2cHandle = roveI2cInit(0, I2CSPEED_FULL, PB_2, PB_3);

  //9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(6, 9600, PP_1, PP_0);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  //Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  //Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
  roveI2cSendReg(i2cHandle, Address_AG, CTRL_REG4,0B00111000);  //enable gyro axis
  roveI2cSendReg(i2cHandle, Address_AG, CTRL_REG5_XL,0B00111000); //enable accelerometer
  roveI2cSendReg(i2cHandle, Address_AG, CTRL_REG1_G,0B01100000); //gyro/accel odr and bw
  roveI2cSendReg(i2cHandle, Address_M, CTRL_REG3_M,0B00000000); //enable mag continuous

}//end

uint32_t timer = millis();

int main()
{
  setup();
  while(1)
  {
    loop();
  }
}//end loop

void loop()
{
  uint16_t data_id = 0;
  size_t data_size = 0;
  uint16_t data = 0;
  roveComm_GetMsg(&data_id, &data_size, &data);
  //delay(300);

  //int16_t msg = 0;
  //roveComm_SendMsg(301, sizeof(msg), &msg);
  //delay(300);

  //Serial.print("Looping");
  //delay(1);

  char c = GPS.read();
  //delay(10);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {

    if (!GPS.parse(GPS.lastNMEA() ) )// this also sets the newNMEAreceived() flag to false
    {
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }//end if

  }//end if

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())
  {
    timer = millis();
  }//end if

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 20) {
    timer = millis(); // reset the timer

    //debug
    /*
    GPS.fix = true;
    GPS.fixquality = 200;
    GPS.latitude_fixed = 407098514;
    GPS.longitude_fixed = -740079168;
    GPS.speed = 123.456;
    GPS.angle = 789.012;
    GPS.altitude = 345.678;
    GPS.satellites = 251;
   */
    if(!GPS.fix)
    {
      GPS.fixquality = 0;
    }//end if

    //Serial.print(" quality: "); Serial.println(GPS.fixquality);
    roveComm_SendMsg(GPS_FIX_QUALITY_DATA_ID, sizeof(GPS.fixquality), &GPS.fixquality);

    if (GPS.fix)
    {
      //TODO: VERIFY ADAFRUIT_GPS PULL #13
      gps_lat_lon = GPS.latitude_fixed;
      gps_lat_lon = gps_lat_lon << 32;
      gps_lat_lon += GPS.longitude_fixed;

      roveComm_SendMsg(GPS_LAT_LON_DATA_ID, sizeof(gps_lat_lon), &gps_lat_lon);

      //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      roveComm_SendMsg(GPS_SPEED_DATA_ID, sizeof(GPS.speed), &GPS.speed);

      //Serial.print("Angle: "); Serial.println(GPS.angle);
      roveComm_SendMsg(GPS_ANGLE_DATA_ID, sizeof(GPS.angle), &GPS.angle);

      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      roveComm_SendMsg(GPS_ALTITUDE_DATA_ID, sizeof(GPS.altitude), &GPS.altitude);

      //Serial.print("Satellites: "); Serial.println(GPS.satellites);
      roveComm_SendMsg(GPS_SATELLITES_DATA_ID, sizeof(GPS.satellites), &GPS.satellites);
    }//end if

  uint8_t X_L; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_X_L_G, &X_L);//gyroscope pitch
  uint8_t X_H; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_X_H_G, &X_H);
  uint8_t Y_L; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Y_L_G, &Y_L);
  uint8_t Y_H; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Y_H_G, &Y_H);
  uint8_t Z_L; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Z_L_G, &Z_L);
  uint8_t Z_H; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Z_H_G, &Z_H);

  int16_t X_AXIs = X_H <<8 | X_L;
  int16_t Y_AXIs = Y_H <<8 | Y_L;
  int16_t Z_AXIs = Z_H <<8 | Z_L;


  float real_X_Axis =0.00875*(X_AXIs-320);
  float real_Y_Axis =0.00875*(Y_AXIs-17);
  float real_Z_Axis =0.00875*(Z_AXIs+190);
  float GYRO_DATA[3] = {real_X_Axis, real_Y_Axis, real_Z_Axis};
  roveComm_SendMsg(IMU_GYRO_DATA_ID, sizeof(GYRO_DATA), GYRO_DATA);



  //accelerometer/magnetometer section
  uint8_t X_L_M; roveI2cReceiveReg(i2cHandle, Address_M, OUT_X_L_M, &X_L_M);//Magnetometer data expressed as two's complement
  uint8_t X_H_M; roveI2cReceiveReg(i2cHandle, Address_M, OUT_X_H_M, &X_H_M);
  uint8_t Y_L_M; roveI2cReceiveReg(i2cHandle, Address_M, OUT_Y_L_M, &Y_L_M);
  uint8_t Y_H_M; roveI2cReceiveReg(i2cHandle, Address_M, OUT_Y_H_M, &Y_H_M);
  uint8_t Z_L_M; roveI2cReceiveReg(i2cHandle, Address_M, OUT_Z_L_M, &Z_L_M);
  uint8_t Z_H_M; roveI2cReceiveReg(i2cHandle, Address_M, OUT_Z_H_M, &Z_H_M);

  uint8_t X_L_A; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_X_L_XL, &X_L_A);//Output acceleration in x-axis as a 16-bit word in two's complement
  uint8_t X_H_A; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_X_H_XL, &X_H_A);
  uint8_t Y_L_A; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Y_L_XL, &Y_L_A);
  uint8_t Y_H_A; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Y_H_XL, &Y_H_A);
  uint8_t Z_L_A; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Z_L_XL, &Z_L_A);
  uint8_t Z_H_A; roveI2cReceiveReg(i2cHandle, Address_AG, OUT_Z_H_XL, &Z_H_A);

  int16_t X_AXIS_M = X_H_M <<8 | X_L_M;
  int16_t Y_AXIS_M = Y_H_M <<8 | Y_L_M;
  int16_t Z_AXIS_M = Z_H_M <<8 | Z_L_M;

  int16_t X_AXIS_A = X_H_A <<8 | X_L_A;
  int16_t Y_AXIS_A = Y_H_A <<8 | Y_L_A;
  int16_t Z_AXIS_A = Z_H_A <<8 | Z_L_A;

  //temperature section
  uint8_t Temp_L;  roveI2cReceiveReg(i2cHandle, Address_AG, OUT_TEMP_L, &Temp_L);
  uint8_t Temp_H;  roveI2cReceiveReg(i2cHandle, Address_AG, OUT_TEMP_H, &Temp_H);

  int16_t Temp = Temp_H <<8 | Temp_L;
  float real_Temp = (Temp/16.0)+25;
  roveComm_SendMsg(IMU_TEMP_DATA_ID, sizeof(real_Temp), &real_Temp);

  float real_X_Axis_M = X_AXIS_M*0.00014;
  float real_Y_Axis_M = Y_AXIS_M*0.00014;
  float real_Z_Axis_M = Z_AXIS_M*0.00014;
  float MAG_DATA[3];
  MAG_DATA[0]= real_X_Axis_M;
  MAG_DATA[1] = real_Y_Axis_M;
  MAG_DATA[2] = real_Z_Axis_M;
  roveComm_SendMsg(IMU_MAG_DATA_ID, sizeof(MAG_DATA), MAG_DATA);


  float real_X_AXIS_A = X_AXIS_A*0.000061;
  float real_Y_AXIS_A = Y_AXIS_A*0.000061;
  float real_Z_AXIS_A = Z_AXIS_A*0.000061;

  float ACCEL_DATA[3];
  ACCEL_DATA[0]= real_X_AXIS_A;
  ACCEL_DATA[1] = real_Y_AXIS_A;
  ACCEL_DATA[2] =real_Z_AXIS_A;

  roveComm_SendMsg(IMU_ACCEL_DATA_ID, sizeof(ACCEL_DATA), ACCEL_DATA);

  }//end if
}
