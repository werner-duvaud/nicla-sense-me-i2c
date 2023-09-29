/*
Need to make _bhy2 public in `Arduino/libraries/Arduino_BHY2/src/BoschSensortec.h`.

Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bhi260ap-ds000.pdf
*/

#include "Arduino_BHY2.h"
#include "Wire.h"

SensorXYZ accelerometer(SENSOR_ID_ACC);  // Accelerometer corrected
SensorXYZ gyroscope(SENSOR_ID_GYRO);  // Gyroscope corrected
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);  // Magnetometer corrected
SensorOrientation orientation(SENSOR_ID_ORI);  // Orientation
SensorQuaternion rotation(SENSOR_ID_RV);  // Rotation vector
Sensor temperature(SENSOR_ID_TEMP);  // Temperature

const uint8_t I2C_ADDRESS = 0x28;

// I2C commands.
// Naming from the point of view of the sensor.
const uint8_t CMD_BEGIN = 0x60;
const uint8_t CMD_SEND_CALIB = 0x5B;
const uint8_t CMD_SEND_ORIENTATION = 0x1A;
const uint8_t CMD_SEND_ACCELEROMETER = 0x08;
const uint8_t CMD_SEND_GYROSCOPE = 0x14;
const uint8_t CMD_SEND_MAGNETOMETER = 0x0E;
const uint8_t CMD_SEND_ROTATION = 0x20;
const uint8_t CMD_SEND_TEMPERATURE = 0x50;
const uint8_t CMD_FETCH_CALIB = 0x5A;
const uint8_t CMD_RECEIVE_CALIB = 0x5D;
const uint8_t CMD_APPLY_CALIB = 0x5E;
const uint8_t CMD_DEFAULT = 0x00;
const uint8_t CMD_DEFAULT_REPLY = 0xFF;

// Store current I2C command.
uint8_t i2c_command = CMD_DEFAULT;

// Store calib accross I2C calls.
uint8_t accCalib[512];
uint8_t gyrCalib[512];
uint8_t magCalib[512];
int currentCalibPart = 0;

void sendCalib() {
  // See datasheet 13.3.3 BSX Algorithm Parameters.
  // Send calib by chunks of 16 bytes.
  uint8_t buffer[sizeof(uint8_t) * 16];
  if (0 <= currentCalibPart && currentCalibPart < 32) {
    memcpy(buffer, &accCalib[currentCalibPart * 16], sizeof(uint8_t) * 16);
  } else if (32 <= currentCalibPart && currentCalibPart < 64) {
    memcpy(buffer, &gyrCalib[(currentCalibPart - 32) * 16], sizeof(uint8_t) * 16);
  } else if (64 <= currentCalibPart && currentCalibPart < 96) {
    memcpy(buffer, &magCalib[(currentCalibPart - 64) * 16], sizeof(uint8_t) * 16);
  }
  Wire.write(buffer, sizeof(buffer));
  
  currentCalibPart ++;

  // Everything has been sent.
  if (currentCalibPart == 96) {
    currentCalibPart = 0;
  }
}

void receiveCalib() {
  // See datasheet 13.3.3 BSX Algorithm Parameters.
  // Receive calib by chunks of 16 bytes.
  for (int i = 0; i < 16; i++) {
    uint8_t receivedByte = Wire.read();

    if (0 <= currentCalibPart && currentCalibPart < 32) {
      accCalib[currentCalibPart * 16 + i] = receivedByte;
    } else if (32 <= currentCalibPart && currentCalibPart < 64) {
      gyrCalib[(currentCalibPart - 32) * 16 + i] = receivedByte;
    } else if (64 <= currentCalibPart && currentCalibPart < 96) {
      magCalib[(currentCalibPart - 64) * 16 + i] = receivedByte;
    }
  }

  currentCalibPart ++;
  
  // Everything has been received.
  if (currentCalibPart == 96) {
    currentCalibPart = 0;
    i2c_command = CMD_DEFAULT;
  }
}

void sendAccelerometer() {
  int16_t accelerationData[3];
  accelerationData[0] = accelerometer.x();
  accelerationData[1] = accelerometer.y();
  accelerationData[2] = accelerometer.z();
  Wire.write((uint8_t*)accelerationData, sizeof(accelerationData));
}

void sendGyroscope() {
  int16_t gyroscopeData[3];
  gyroscopeData[0] = gyroscope.x();
  gyroscopeData[1] = gyroscope.y();
  gyroscopeData[2] = gyroscope.z();
  Wire.write((uint8_t*)gyroscopeData, sizeof(gyroscopeData));
}

void sendMagnetometer() {
  int16_t magnetometerData[3];
  magnetometerData[0] = magnetometer.x();
  magnetometerData[1] = magnetometer.y();
  magnetometerData[2] = magnetometer.z();
  Wire.write((uint8_t*)magnetometerData, sizeof(magnetometerData));
}

void sendOrientation() {
  int16_t orientationData[3];
  orientationData[0] = static_cast<int16_t>(orientation.heading() * 91);
  orientationData[1] = static_cast<int16_t>(orientation.roll() * 91);
  orientationData[2] = static_cast<int16_t>(orientation.pitch() * 91);
  Wire.write((uint8_t*)orientationData, sizeof(orientationData));
}

void sendRotation() {
  int16_t rotationData[4];
  rotationData[0] = static_cast<int16_t>(rotation.x() * 32767);
  rotationData[1] = static_cast<int16_t>(rotation.y() * 32767);
  rotationData[2] = static_cast<int16_t>(rotation.z() * 32767);
  rotationData[3] = static_cast<int16_t>(rotation.w() * 32767);
  Wire.write((uint8_t*)rotationData, sizeof(rotationData));
}

void sendTemperature() {
  int16_t temperatureData[1];
  temperatureData[0] = static_cast<int16_t>(temperature.value() * 100);
  Wire.write((uint8_t*)temperatureData, sizeof(temperatureData));
}

void fetchCalib () {
  // Fetch calib from BSX.
  // See datasheet 13.3.3 BSX Algorithm Parameters.
  uint32_t actual_len;
  bhy2_get_calibration_profile(1, accCalib, 512, &actual_len, &sensortec._bhy2);
  bhy2_get_calibration_profile(3, gyrCalib, 512, &actual_len, &sensortec._bhy2);
  bhy2_get_calibration_profile(5, magCalib, 512, &actual_len, &sensortec._bhy2);
}

void applyCalib() {
  // Set calib in BSX.
  // See datasheet 13.3.3 BSX Algorithm Parameters.
  bhy2_set_calibration_profile(1, accCalib, 72, &sensortec._bhy2);
  bhy2_set_calibration_profile(3, gyrCalib, 200, &sensortec._bhy2);
  bhy2_set_calibration_profile(5, magCalib, 408, &sensortec._bhy2);
}

void receiveEvent(int numBytes) {
  if (numBytes == 1) {
    // Receive a command.
    i2c_command = Wire.read();
  } else if (numBytes == 16 && i2c_command == CMD_RECEIVE_CALIB) {
    // Receive calib data.
    receiveCalib();
  }
}

void requestHandler() {
  if (i2c_command == CMD_SEND_CALIB) {
    sendCalib();
  } else if (i2c_command == CMD_SEND_ORIENTATION) {
    sendOrientation();
  } else if (i2c_command == CMD_SEND_ACCELEROMETER) {
    sendAccelerometer();
  } else if (i2c_command == CMD_SEND_GYROSCOPE) {
    sendGyroscope();
  } else if (i2c_command == CMD_SEND_MAGNETOMETER) {
    sendMagnetometer();
  } else if (i2c_command == CMD_SEND_ROTATION) {
    sendRotation();
  } else if (i2c_command == CMD_SEND_TEMPERATURE) {
    sendTemperature();
  } else {
    // requestHandler is expected to always write.
    // Also makes the device show up in linux i2cdetect.
    Wire.write(CMD_DEFAULT_REPLY);
  }

  // Command executed. Reset to default.
  i2c_command = CMD_DEFAULT;
}

void begin() {
  // Can't change the calibration manually after calling this function.
  accelerometer.begin();
  gyroscope.begin();
  magnetometer.begin();
  orientation.begin();
  rotation.begin();
  temperature.begin();
}

void setup() {
  // Init
  BHY2.begin(NICLA_STANDALONE);
  Wire.begin(I2C_ADDRESS);

  // I2C callbacks.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestHandler);
}

void loop() {
  BHY2.update();

  // Some commands take too long to be applied directly within the I2C callback.
  if (i2c_command == CMD_BEGIN) {
    begin();
    i2c_command = CMD_DEFAULT;
  } else if (i2c_command == CMD_FETCH_CALIB) {
    fetchCalib();
    i2c_command = CMD_DEFAULT;
  } else if (i2c_command == CMD_APPLY_CALIB) {
    applyCalib();
    i2c_command = CMD_DEFAULT;
  }
}
