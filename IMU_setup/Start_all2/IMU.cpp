#include "IMU.h"
ICM_20948_I2C myICM;  // Create an ICM_20948_I2C object

void IMU_setup() {
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  int retryCount = 0;
  while (!initialized && retryCount < 10) {
    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println(F("Retrying IMU initialization..."));
      delay(500);
    } else {
      initialized = true;
    }
    retryCount++;
  }

  if (!initialized) {
    SERIAL_PORT.println(F("IMU failed to initialize after 10 retries."));
    return;
  }

  SERIAL_PORT.println(F("Device connected!"));

  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  myICM.sleep(false);
  myICM.lowPower(false);

  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  ICM_20948_fss_t myFSS;
  myFSS.a = gpm2;
  myFSS.g = dps500;

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  ICM_20948_dlpcfg_t myDLPcfg;
  myDLPcfg.a = acc_d473bw_n499bw;
  myDLPcfg.g = gyr_d361bw4_n376bw5;

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  myICM.enableDLPF(ICM_20948_Internal_Gyr, false);

  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok) {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  SERIAL_PORT.println(F("Configuration complete!"));
}

void readIMU(float &x_acc, float &y_acc, float &z_acc, float &x_gyr, float &y_gyr, float &z_gyr, float &x_mag, float &y_mag, float &z_mag) {
  if (myICM.dataReady()) {
    myICM.getAGMT();  // Fetch IMU data

    x_acc = myICM.accX();
    y_acc = myICM.accY();
    z_acc = myICM.accZ();

    x_gyr = myICM.gyrX();
    y_gyr = myICM.gyrY();
    z_gyr = myICM.gyrZ();

    x_mag = myICM.magX();
    y_mag = myICM.magY();
    z_mag = myICM.magZ();

    printScaledAGMT(x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr, x_mag, y_mag, z_mag);  // Use updated print function
  } else {
    SERIAL_PORT.println("Waiting for data");
  }
}

void printScaledAGMT(float x_acc, float y_acc, float z_acc, float x_gyr, float y_gyr, float z_gyr, float x_mag, float y_mag, float z_mag) {
  printFormattedFloat(x_acc, 5, 2, "Accel_X");
  SERIAL_PORT.print(" ");
  printFormattedFloat(y_acc, 5, 2, "Accel_Y");
  SERIAL_PORT.print(" ");
  printFormattedFloat(z_acc, 5, 2, "Accel_Z");
  SERIAL_PORT.print(" ");
  printFormattedFloat(x_gyr, 5, 2, "Gyro_X");
  SERIAL_PORT.print(" ");
  printFormattedFloat(y_gyr, 5, 2, "Gyro_Y");
  SERIAL_PORT.print(" ");
  printFormattedFloat(z_gyr, 5, 2, "Gyro_Z");
  SERIAL_PORT.print(" ");
  printFormattedFloat(x_mag, 5, 2, "Mag_X");
  SERIAL_PORT.print(" ");
  printFormattedFloat(y_mag, 5, 2, "Mag_Y");
  SERIAL_PORT.print(" ");
  printFormattedFloat(z_mag, 5, 2, "Mag_Z");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals, const char *label) {
  SERIAL_PORT.print(label);
  SERIAL_PORT.print(":");
  float aval = abs(val);
  if (val < 0) {
    SERIAL_PORT.print("-");
  } else {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      SERIAL_PORT.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  } else {
    SERIAL_PORT.print(val, decimals);
  }
}

// void printScaledAGMT1(ICM_20948_I2C *sensor){
//   SERIAL_PORT.print("Scaled. Acc (mg) [ ");
//   printFormattedFloat(sensor->accX(), 5, 2, "Accel_X");
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->accY(), 5, 2, "Accel_Y");
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->accZ(), 5, 2, "Accel_Z");
//   SERIAL_PORT.print(" ], Gyr (DPS) [ ");
//   printFormattedFloat(sensor->gyrX(), 5, 2, "Gyro_X");
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->gyrY(), 5, 2, "Gyro_Y");
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->gyrZ(), 5, 2, "Gyro_Z");
//   SERIAL_PORT.print(" ], Mag (uT) [ ");
//   printFormattedFloat(sensor->magX(), 5, 2, "Mag_X");
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->magY(), 5, 2, "Mag_Y");
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->magZ(), 5, 2, "Mag_Z");
//   SERIAL_PORT.print(" ], Tmp (C) [ ");
//   printFormattedFloat(sensor->temp(), 5, 2, "Temp");
//   SERIAL_PORT.print(" ]");
//   SERIAL_PORT.println();
// }
