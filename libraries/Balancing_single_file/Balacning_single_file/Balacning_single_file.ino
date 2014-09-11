#include <Wire.h>
#include <Kalman.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))
                         
int L3G4200D_Address = 105; 


//Set up kalman instances
Kalman kalmanPitch; 
Kalman kalmanRoll;


//Set up accelerometer variables
int16_t accValX, accValY, accValZ;
float accBiasX, accBiasY, accBiasZ;
float accAngleX, accAngleY;
double accPitch, accRoll;

//Set up gyroscope variables
int16_t gyroX, gyroY, gyroZ;
float gyroBiasX, gyroBiasY, gyroBiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
float gyroBias_oldX, gyroBias_oldY, gyroBias_oldZ;
float gyroPitch = 180;
float gyroRoll = -180;
float gyroYaw = 0;
double gyro_sensitivity = 70; //From datasheet, depends on Scale, 2000DPS = 70, 500DPS = 17.5, 250DPS = 8.75. 


int x;
int y;
int z;
//Set up a timer Variable
uint32_t timer;

// valores angulos
double InputPitch, InputRoll;

// Valores iniciales
double PitchInicial,RollInicial;

// Motores
int enablea = 3;
int enableb = 9;
int a1 = 5;
int a2 = 4;
int b1 = 10;
int b2 = 11;


void setup() {
  
  Wire.begin();

  Serial.begin(9600);
  setupL3G4200D(200); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); 

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  
    // Motor
  pinMode(enablea, OUTPUT);  
  pinMode(enableb, OUTPUT); 
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);  
  pinMode(b1, OUTPUT);  
  pinMode(b2, OUTPUT); 
 
  digitalWrite(a1,HIGH); 
  digitalWrite(a2,HIGH); 
  digitalWrite(b1,HIGH); 
  digitalWrite(b2,HIGH); 
  
  accel.setRange(ADXL345_RANGE_2_G);
  sensors_event_t event; 
  accel.getEvent(&event);
  
  // Calculate bias for the Gyro i.e. the values it gives when it's not moving
  for(int i=1; i < 100; i++){
    
    getGyroValues();
    gyroBiasX += (int)x;
    gyroBiasY += (int)y;
    gyroBiasZ += (int)z;  

    accel.getEvent(&event);
    accBiasX += event.acceleration.x;
    accBiasY += event.acceleration.y;
    accBiasZ += event.acceleration.z;

    delay(1);
  }
  

  gyroBiasX = gyroBiasX / 100;
  gyroBiasY = gyroBiasY / 100;
  gyroBiasZ = gyroBiasZ / 100;

  accBiasX = accBiasX / 100;
  accBiasY = accBiasY / 100;
  accBiasZ = accBiasZ / 100;
  

  //Get Starting Pitch and Roll
  accel.getEvent(&event);
  accPitch = (atan2(-event.acceleration.x,-event.acceleration.z)+PI)*RAD_TO_DEG;
  accRoll = (atan2(event.acceleration.y,-event.acceleration.z)+PI)*RAD_TO_DEG;

  if (accPitch <= 360 & accPitch >= 180){
    accPitch = accPitch - 360;
  }

  if (accRoll <= 360 & accRoll >= 180){
    accRoll = accRoll - 360;
  }
  

  // Set starting angle for Kalman
  kalmanPitch.setAngle(accPitch); 
  kalmanRoll.setAngle(accRoll);
  
  kalmanRoll.setQangle(0.01);      // 0.001
  kalmanRoll.setQbias(0.0003);    // 0.003
  kalmanRoll.setRmeasure(0.01);    // 0.03
  
  gyroPitch = accPitch;
  gyroRoll = accRoll;

  timer = micros();
  delay(1000);
  ValoresIniciales();

}

double tau=0.075;
double a=0.0;
double x_angleC = 0;

double Complementary(double newAngle, double newRate,double looptime) {
  
  double dtC = float(looptime)/1000000.0;
  a=tau/(tau+dtC);
  x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
  return x_angleC;
  
}

double Setpoint;

void MotorControl(double out){
  
  if (out > 0){
    digitalWrite(a1,HIGH);
    digitalWrite(a2,LOW);
    digitalWrite(b1,LOW);
    digitalWrite(b2,HIGH);
  }else{
    digitalWrite(a1,LOW);
    digitalWrite(a2,HIGH);
    digitalWrite(b1,HIGH);
    digitalWrite(b2,LOW);
  }
  
  byte vel = abs(out);
  if (vel<0)
    vel=0;
  if (vel > 255)
    vel=255;
  
  //Serial.println(out);
  analogWrite(enablea,vel);
  analogWrite(enableb,vel);
}

void ValoresIniciales(){
  
  //////////////////////
  //  Accelerometer   //
  //////////////////////
  sensors_event_t event; 

  accel.getEvent(&event);
  accPitch = (atan2(-event.acceleration.x,-event.acceleration.z)+PI)*RAD_TO_DEG;
  accRoll = (atan2(event.acceleration.y,-event.acceleration.z)+PI)*RAD_TO_DEG;

  if (accPitch <= 360 & accPitch >= 180){
    accPitch = accPitch - 360;
  }

  if (accRoll <= 360 & accRoll >= 180){
    accRoll = accRoll - 360;
  }


  //////////////////////
  //      GYRO        //
  //////////////////////


  getGyroValues();

  // read raw angular velocity measurements from device  
  gyroRateX = ((int)x - gyroBiasX)*.07; //*(.0105);
  gyroRateY = -((int)y - gyroBiasY)*.07; //*(.0105);
  gyroRateZ = ((int)z - gyroBiasZ)*.07; //*(.0105);

  gyroPitch += gyroRateY * ((double)(micros() - timer)/1000000);
  gyroRoll += gyroRateX * ((double)(micros() - timer)/1000000);
  gyroYaw += gyroRateZ * ((double)(micros() - timer)/1000000);

  PitchInicial = kalmanPitch.getAngle(accPitch, gyroPitch, (double)(micros()-timer)/1000000);
  //RollInicial = kalmanRoll.getAngle(accRoll, gyroRoll, (double)(micros()-timer)/1000000);
  timer = micros();
  RollInicial = gyroRoll;
  
  Serial.print("Pitch Inicial: ");
  Serial.println(PitchInicial);
  Serial.print("Roll Inicial: ");
  Serial.println(RollInicial);
  Setpoint = 0;
  
}

int i=0;
double aaa =0;
void loop() {
  
  runEvery(10){
    //////////////////////
    //  Accelerometer   //
    //////////////////////
    sensors_event_t event; 
  
    accel.getEvent(&event);
    accPitch = (atan2(-event.acceleration.x,-event.acceleration.z)+PI)*RAD_TO_DEG;
    accRoll = (atan2(event.acceleration.y,-event.acceleration.z)+PI)*RAD_TO_DEG;
  
    if (accPitch <= 360 & accPitch >= 180){
      accPitch = accPitch - 360;
    }
  
    if (accRoll <= 360 & accRoll >= 180){
      accRoll = accRoll - 360;
    }
  
  
    //////////////////////
    //      GYRO        //
    //////////////////////
  
  
    getGyroValues();
  
    // read raw angular velocity measurements from device  
    gyroRateX = -((int)x - gyroBiasX)*.07; //*(.0105);
    gyroRateY = -((int)y - gyroBiasY)*.07; //*(.0105);
    gyroRateZ = ((int)z - gyroBiasZ)*.07; //*(.0105);
  
    gyroPitch += gyroRateY * ((double)(micros() - timer)/1000000);
    double leeeeel = gyroRateX * ((double)(micros() - timer)/1000000);
    gyroRoll += gyroRateX * ((double)(micros() - timer)/1000000);
    gyroYaw += gyroRateZ * ((double)(micros() - timer)/1000000);
  
    InputPitch = kalmanPitch.getAngle(accPitch, gyroPitch, (double)(micros()-timer)/1000000);
    InputRoll = kalmanRoll.getAngle(accRoll, gyroRoll, (double)(micros()-timer)/1000000);
    timer = micros();
    
    
    //angle = (0.98)*(angle + gyroRateX * dt) + (0.02)*(accRoll);
  
    //Serial.write(57);
    //Serial.write((byte)accRoll);
    byte a= map(abs(Compute(InputRoll-RollInicial)),0,255,0,124);
    Serial.println(Setpoint);
    //Serial.write((byte)(InputRoll-RollInicial));
    //Serial.write((byte)accRoll);
    //Serial.write((byte)gyroRoll);
    aaa = 0.98* (aaa + leeeeel) + 0.02 * (accRoll);
    //Serial.write((byte)aaa);
    //Serial.write((byte)InputPitch);
    //Serial.write((byte)gyroRateY);
    //Serial.write((byte)InputRoll);
    //Serial.write((byte)heading);
    //MostrarDatos();
    MotorControl(Compute(aaa-RollInicial));
    
   // i++;
    //if (i=100){
      //i=0;
      //gyroRoll = (InputRoll-RollInicial);
    //}  
      
  }
  
  if (Serial.available()){
    char a = Serial.read();
    switch (a){
      case 'q':
        Setpoint += 0.01;
      break;
      case 'w':
        Setpoint -= 0.01;
      break;
      case 'e':
        Setpoint += 0.1;
      break;
      case 'r':
        Setpoint -= 0.1;
      break;
    } 
  }
  
}

void MostrarDatos(){
 
  Serial.print("DATOS: ");
  Serial.print("RollInicial: ");
  Serial.print(RollInicial);
  Serial.print("Roll: ");
  Serial.print(InputRoll);
  Serial.print("RollBueno: ");
  Serial.println(InputRoll-RollInicial);
  
}


int outMax = 255;
int outMin = -255;
float lastInput = 0;
double ITerm =0;
double kp = 100;
double ki = 2;
double kd = 0;

double Compute(double input)
{

      double error = Setpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      /*Compute PID Output*/
      double output = kp * error + ITerm + kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      return output;
}


void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}




