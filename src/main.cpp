// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


void waitSerialAvailable(void)
{
  while (!Serial.available())
  {
    delay(10);
  }
}

void userMpuConfig(void)
{
  Serial.println("config");

  char input;

  waitSerialAvailable();
  input = Serial.read();
  if (input == '1')
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  }
  else if (input == '2')
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  }
  else if (input == '3')
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  }
  else if (input == '4')
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  }

  waitSerialAvailable();
  input = Serial.read();
  if (input == '1')
  {
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  }
  else if (input == '2')
  {
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  }
  else if (input == '3')
  {
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  }
  else if (input == '4')
  {
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  }

  waitSerialAvailable();
  input = Serial.read();
  if (input == '1')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  }
  else if (input == '2')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  }
  else if (input == '3')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  }
  else if (input == '4')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
  else if (input == '5')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  else if (input == '6')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  }
  else if (input == '7')
  {
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  }
}

void printMpuStatus(void)
{
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void setup(void)
{
  Serial.begin(115200);
  delay(1000);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.flush();
  Serial.println();

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  userMpuConfig();
  printMpuStatus();

  delay(100);
  Serial.print("Timespan (ms)");
  Serial.print(";");
  Serial.print("Accel X (m/s^2)");
  Serial.print(";");
  Serial.print("Accel Y (m/s^2)");
  Serial.print(";");
  Serial.print("Accel Z (m/s^2)");
  Serial.print(";");
  Serial.print("Gyro X (rad/s)");
  Serial.print(";");
  Serial.print("Gyro Y (rad/s)");
  Serial.print(";");
  Serial.print("Gyro Z (rad/s)");
  Serial.print(";");
  Serial.println("Temperature (degC)");
}

void loop()
{

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print(millis());
  Serial.print(";");
  Serial.print(a.acceleration.x);
  Serial.print(";");
  Serial.print(a.acceleration.y);
  Serial.print(";");
  Serial.print(a.acceleration.z);
  Serial.print(";");

  Serial.print(g.gyro.x);
  Serial.print(";");
  Serial.print(g.gyro.y);
  Serial.print(";");
  Serial.print(g.gyro.z);

  Serial.print(";");
  Serial.println(temp.temperature);
}