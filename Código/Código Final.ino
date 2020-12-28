#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu1;
MPU6050 mpu2(0x69);

#define int1 15//D8
#define int2 13//D7

bool dmpReady1 = false;
uint8_t mpuIntStatus1;
uint8_t devStatus1;
uint16_t packetSize1;
uint16_t fifoCount1;
uint8_t fifoBuffer1[64];

bool dmpReady2 = false;
uint8_t mpuIntStatus2;
uint8_t devStatus2;
uint16_t packetSize2;
uint16_t fifoCount2;
uint8_t fifoBuffer2[64];

Quaternion q1;
float euler1[3];// [psi, theta, phi]

Quaternion q2;
float euler2[3];// [psi, theta, phi]

volatile bool mpuInterrupt1 = false;
void dmpDataReady1() {
  mpuInterrupt1 = true;
}

volatile bool mpuInterrupt2 = false;
void dmpDataReady2() {
  mpuInterrupt2 = true;
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(5,4);//D1, D2  sda,scl
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu1.initialize();
  mpu2.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
  Serial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());
  
  Serial.println(F("Initializing DMPs..."));
  devStatus1 = mpu1.dmpInitialize();
  devStatus2 = mpu2.dmpInitialize();
  
  mpu1.setXGyroOffset(54);
  mpu1.setYGyroOffset(-9);
  mpu1.setZGyroOffset(7);
  mpu1.setXAccelOffset(-1205);
  mpu1.setYAccelOffset(-1585);
  mpu1.setZAccelOffset(1067);

  mpu2.setXGyroOffset(11);
  mpu2.setYGyroOffset(67);
  mpu2.setZGyroOffset(30);
  mpu2.setXAccelOffset(-1575);
  mpu2.setYAccelOffset(3210);
  mpu2.setZAccelOffset(1984);


  if (devStatus1 == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(int1));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(int1), dmpDataReady1, RISING);
    mpuIntStatus1 = mpu1.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady1 = true;
    packetSize1 = mpu1.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus1);
    Serial.println(F(")"));
  }


  if (devStatus2 == 0) {
    Serial.println(F("Enabling DMP2..."));
    mpu2.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection 2 (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(int2));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(int2), dmpDataReady2, RISING);
    mpuIntStatus2 = mpu2.getIntStatus();

    Serial.println(F("DMP2 ready! Waiting for first interrupt..."));
    dmpReady2 = true;
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP2 Initialization failed (code "));
    Serial.print(devStatus2);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady1) return;
  while (!mpuInterrupt1 && fifoCount1 < packetSize1) {
    if (mpuInterrupt1 && fifoCount1 < packetSize1) {
      fifoCount1 = mpu1.getFIFOCount();
    }
  }
  mpuInterrupt1 = false;
  mpuIntStatus1 = mpu1.getIntStatus();
  fifoCount1 = mpu1.getFIFOCount();
  if ((mpuIntStatus1 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount1 >= 1024) {
    mpu1.resetFIFO();
    fifoCount1 = mpu1.getFIFOCount();
  } else if (mpuIntStatus1 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount1 < packetSize1) fifoCount1 = mpu1.getFIFOCount();
    mpu1.getFIFOBytes(fifoBuffer1, packetSize1);
    fifoCount1 -= packetSize1;
    mpu1.dmpGetQuaternion(&q1, fifoBuffer1);
    mpu1.dmpGetEuler(euler1, &q1);
    Serial.print(euler1[0] * 180 / M_PI);
    Serial.print(", ");
    Serial.print(euler1[1] * 180 / M_PI);
    Serial.print(", ");
    Serial.print(euler1[2] * 180 / M_PI);
    Serial.print(", ");
    delay(1000);
  }

  //----------------------------------------------------------------
  if (!dmpReady2) return;
  while (!mpuInterrupt2 && fifoCount2 < packetSize2) {
    if (mpuInterrupt2 && fifoCount2 < packetSize2) {
      fifoCount2 = mpu2.getFIFOCount();
    }
  }
  mpuInterrupt2 = false;
  mpuIntStatus2 = mpu2.getIntStatus();
  fifoCount2 = mpu2.getFIFOCount();
  if ((mpuIntStatus2 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount2 >= 1024) {
    mpu2.resetFIFO();
    fifoCount2 = mpu2.getFIFOCount();
  } else if (mpuIntStatus2 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount2 < packetSize2) fifoCount2 = mpu2.getFIFOCount();
    mpu2.getFIFOBytes(fifoBuffer2, packetSize2);
    fifoCount2 -= packetSize2;
    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
    mpu2.dmpGetEuler(euler2, &q2);
    Serial.print(euler2[0] * 180 / M_PI);
    Serial.print(", ");
    Serial.print(euler2[1] * 180 / M_PI);
    Serial.print(", ");
    Serial.print(euler2[2] * 180 / M_PI);
    Serial.println("");
    delay(1000);
  }
}
