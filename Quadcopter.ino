#include <Servo.h>

//Create the 4 esc objects
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
double initial_x = 0;
double initial_y = 0;
//Esc pins

int escPin1 = 8;
int escPin2 = 9;
int escPin3 = 10;
int escPin4 = 11;
int pid_Output_x;
int pid_Output_y;
float k_p = 7;
float k_d = .3;
float filter_gain = 0;
int a = 0;

int minPulseRate = 1000;
int maxPulseRate = 2000;

void write_throttle(int esc1_throttle, int esc2_throttle, int esc3_throttle, int esc4_throttle);
/////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
/////////////(pid)///////////////////////////////////////////////////////////////////////////////////
#include <PID_v1.h>

double xSetpoint, xInput, xOutput;
double ySetpoint, yInput, yOutput;

PID xPID(&xInput, &xOutput, &xSetpoint, 1.25, 0.74, 0.24, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, 1.25, 0.74, 0.24, DIRECT);
//1.15,0.68,.24
/////////////////////(analog)////////////////////////////////////////////////////////////////////
float PValue, IValue, DValue;

////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    //Init escs
    esc1.attach(escPin1);
    esc2.attach(escPin2);
    esc3.attach(escPin3);
    esc4.attach(escPin4);
    write_throttle(minPulseRate, minPulseRate, minPulseRate, minPulseRate);

    Serial.begin(57600);
    Serial.println("to Run the program enter R");
    Serial.println("to calibrate escs enter C");
    while (Serial.available() < 1);

    if (Serial.read() == 'C') { calibrate_escs(); }
    if (Serial.read() == 'R') {}
//////////////(PID)////////////////////////////////////////////////////////////////////////
    xSetpoint = 0;

    //tell the PID to range between 0 and the full window size
    xPID.SetOutputLimits(-250, 250);
    //turn the PID on
    xPID.SetMode(AUTOMATIC);
//////////////////(MPU6050)////////////////////////////////////////////////////////////////////
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }
    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();

//////////////////////////////////////////////////////////////////////////////////////////////

}

void loop() {
    for (a = a; a < 10000; a++) {
        if (abs(accX - initial_x) > 2000)
            filter_gain = 1;
        else
            filter_gain = 0.1;

        if (abs(accX - initial_y) > 2000)
            filter_gain = 1;
        else
            filter_gain = 0.1;

        initial_x = initial_x + filter_gain * (accX - initial_x);
        initial_y = initial_y + filter_gain * (accY - initial_y);
    }
    if (abs(gyroX - initial_x) > 1000)
        filter_gain = 1;
    else
        filter_gain = 0.1;

    if (abs(gyroX - initial_y) > 1000)
        filter_gain = 1;
    else
        filter_gain = 0.1;

    initial_x = initial_x + filter_gain * (gyroX - initial_x);

    initial_y = initial_y + filter_gain * (accY - initial_y);
    analog();
    mpu6050_readData();
    xInput = kalAngleX;
    //if (kalAngleX<2 & kalAngleX>-2){
    xSetpoint = 10;
    //else{Setpoint =0;}
    pid_Output_x = k_p * kalAngleX + k_d * initial_x / 10;
    pid_Output_y = k_p * kalAngleY + k_d * initial_y / 10;
    xPID.Compute();
    //xPID = 
    if (pid_Output_x >= 150) {
        pid_Output_x = 150;
    }
    if (pid_Output_x <= -150) {
        pid_Output_x = -150;

    }

    if (pid_Output_y >= 150) {
        pid_Output_y = 150;
    }
    if (pid_Output_y <= -150) {
        pid_Output_y = -150;

    }

    write_throttle(1250,1250,1250,1250);
    Serial.print("  pid_Output_x =  ");
    Serial.print(pid_Output_x);

    Serial.print("\r\n");
}

void arm_escs() {}

void calibrate_escs() {
    //Serial.println("Now to calibrate the ESC do th followinng instructions/r/n");
    Serial.println("Turn off escs power source press any key.");
    while (Serial.available() < 1) {}
    Serial.read();
    write_throttle(maxPulseRate, maxPulseRate, maxPulseRate, maxPulseRate);
    Serial.println("Turn on power source, then wait 2 seconds and press any key.");
    while (Serial.available() < 1) {}
    Serial.read();
    write_throttle(minPulseRate, minPulseRate, minPulseRate, minPulseRate);
    Serial.println("calibrated");
    //write_throttle(1000 ,1000 ,1000,1000);
    delay(2000);
}

//Change velocity of the 4 escs at the same time
void write_throttle(int esc1_throttle, int esc2_throttle, int esc3_throttle, int esc4_throttle) {
    esc1.writeMicroseconds(esc1_throttle);
    esc2.writeMicroseconds(esc2_throttle);
    esc3.writeMicroseconds(esc3_throttle);
    esc4.writeMicroseconds(esc4_throttle);
}

void mpu6050_readData() {

    /* Update all the values */
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double) (micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

    // Serial.print("kalAngleX");Serial.print(kalAngleX); Serial.print("\t");

    // Serial.print("\t");

    // Serial.print("kalAngley");Serial.print(kalAngleY); Serial.print("\t");
}

void analog() {
    PValue = analogRead(A2);
    PValue /= 100;
    IValue = analogRead(A1);
    IValue /= 100;
    DValue = analogRead(A0);
    DValue /= 100;
    xPID.SetTunings(PValue, IValue, DValue);
    // Serial.print("P="),Serial.print(PValue); Serial.print("\t");

    ///Serial.print("\t");

    //Serial.print("I="),Serial.print(IValue); Serial.print("\t");

    // Serial.print("\t");

    // Serial.print("D="),Serial.print(DValue); Serial.print("\t");

}
