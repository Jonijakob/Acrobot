#include <PID_v1.h>



#include <SimpleKalmanFilter.h>

#include <MPU6050_tockn.h>
#include <Wire.h>


#define enA 10
#define in1 6
#define in2 7
#define enB 9
#define in3 5
#define in4 4


MPU6050 mpu6050(Wire);
double Setpoint, Input, Output=0;
double Kp = 45, Ki = 70, Kd =3;
/*double aggKp=12, aggKi=10, aggKd=1;
double consKp=5, consKi=0, consKd=0;
*/
PID stable(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


SimpleKalmanFilter kf =SimpleKalmanFilter (5, 5, 1);
//long timer = -0.05;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(-1.18, -0.13, -0.13);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Setpoint = -0.1;
stable.SetMode(AUTOMATIC);

 stable.SetSampleTime(1);
 stable.SetOutputLimits(-250, 250);





}

void loop() {
  mpu6050.update(); 
       
  double mpuget =mpuget*(1-0.01)+(mpu6050.getGyroAngleY())*0.01;
  double angle= angle*(1-0.01)+(mpu6050.getAccX())*0.01;
  Input = kf.updateEstimate((-mpuget*100)*0.1649);
  
  
   stable.Compute();
 

  //if(millis() - timer > 1000){
  if (0 > Output) {
    
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, abs(Output));
    digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);
    analogWrite(enB, abs(Output));

  }
  else if (Output > 0) {
  
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, abs(Output));
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, abs(Output));

  }
  /*else
  {
    
    analogWrite(enA, 0);
    
    analogWrite(enB, 0);
  }*/
  //Serial.print("angale:");
  //Serial.print("$");
  Serial.print(mpuget);
  // Serial.println(";");
  Serial.print("\t");
//Serial.print("gyro:");
// Serial.print(Input);
  //Serial.print("Output:");
  Serial.println(Output);

  //timer = millis();
  //}

}

