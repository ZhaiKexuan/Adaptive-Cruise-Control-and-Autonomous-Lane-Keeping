#define trigPin1 5
#define echoPin1 4
#define trigPin2 3
#define echoPin2 2
#define trigPin3 9
#define echoPin3 8
#define trigPin4 7
#define echoPin4 6
#include <Servo.h> //define the servo library

Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

//initialize measured time variables
float t1;
float t2;

float t3;
float t4;

int check;

//initialize kalman variables and covariances
float yk = 0.0; //predicted state estimate
float yk1 = 0.0; //initialized updated state estimate for correction 1
float yk2 = 0.0; //initialized updated state estimate for correction 2
float R1 = 0.01; //measurement confidence for correction 1 (calculated offline)
float R2 = 0.01; //measurement confidence for correction 2 (calculated offline)
float P = 100000; //predicted error covariance
float P1 = 0.0; //initialized updated error covariance for correction 1
float P2 = 0.0; //initialized updated error covariance for correction 2
float K = 0.0; //kalman gain
float Q = 1; // Process Covarience

float t_yk = 0.0; //predicted state estimate
float t_yk1 = 0.0; //initialized updated state estimate for correction 1
float t_yk2 = 0.0; //initialized updated state estimate for correction 2
float t_R1 = 0.01; //measurement confidence for correction 1 (calculated offline)
float t_R2 = 0.01; //measurement confidence for correction 2 (calculated offline)
float t_P = 100000; //predicted error covariance
float t_P1 = 0.0; //initialized updated error covariance for correction 1
float t_P2 = 0.0; //initialized updated error covariance for correction 2
float t_K = 0.0; //kalman gain
float t_Q = 1; // Process Covarience

//calibration coefficients
float a1 = 35.486; //determined by offline calibration //sensor 1 in sharpie marks
float b1 = -14.486; //determined by offline calibration //sensor 1 in sharpie marks
float a2 = 36.623; //determined by offline calibration //sensor 3 in sharpie marks
float b2 = -25.741; //determined by offline calibration //sensor 3 in sharpie marks
float a3 = 35.871; //determined by offline calibration //sensor 1 in sharpie marks
float b3 = -10.939; //determined by offline calibration //sensor 1 in sharpie marks
float a4 = 35.262; //determined by offline calibration //sensor 3 in sharpie marks
float b4 = -11.864; //determined by offline calibration //sensor 3 in sharpie marks


int steering = 90, throttle = 1500; //defining global variables to use later

int uk = 1500; //System input (initial value could be adjusted)
float uk_1 = 0;

int t_uk = 90; //System input (initial value could be adjusted)
float t_uk_1 = 0;

float ek = 0; //Measured error (initial value could be adjusted)
float ek_1 = 0;
float ek_2 = 0;

float t_ek = 0; //Measured error (initial value could be adjusted)
float t_ek_1 = 0;
float t_ek_2 = 0;

float kp = 1; // P needs to be adjusted
float ki = .3; // I needs to be adjusted
float kd = .5; // D needs to be adjusted
float k1 = kp + ki + kd;
float k2 = -kp - 2 * kd;
float k3 = kp;

float t_kp = 1; // P needs to be adjusted
float t_ki = .3; // I needs to be adjusted
float t_kd = .5; // D needs to be adjusted
float t_k1 = kp + ki + kd;
float t_k2 = -kp - 2 * kd;
float t_k3 = kp;

float L0 = 380; // ACC reference distance = 300 mm
float L; // ACC system output

float D0 = 280; //
float D; //

void setup() {

  Serial.begin (9600); //serial communications at 9600 bps
  pinMode(trigPin1, OUTPUT); //set trigPin1 as output
  pinMode(echoPin1, INPUT); //set echoPin1 as input
  pinMode(trigPin2, OUTPUT); //set trigPin2 as output
  pinMode(echoPin2, INPUT); //set echoPin2 as input

  pinMode(trigPin3, OUTPUT); //set trigPin3 as output
  pinMode(echoPin3, INPUT); //set echoPin3 as input
  pinMode(trigPin4, OUTPUT); //set trigPin4 as output
  pinMode(echoPin4, INPUT); //set echoPin4 as input

  //Serial.begin(9600); //start serial connection. Uncomment for PC
  ssm.attach(10);    //define that ssm is connected at pin 10
  esc.attach(11);     //define that esc is connected at pin 11
}

void loop() {

  float duration1, distance1, duration2, distance2, duration3, distance3, duration4, distance4; //initialize measurement variables

  //send and receive sound waves from ultrasonic sensor 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH); //read high pulse duration of sensor 1

  t1 = duration1 / 200; //convert duration of sensor 1 into total time value in microseconds
  distance1 = a1 * t1 + b1; //use calibration coefficients of sensor 1 to calculate distance in mm

  //sensor fusion
  //Kalman filter prediction stage
  yk = yk2; //use the first distance as prediction
  P1 = P + Q; //predicted state covariance

  //send and receive sound waves from ultrasonic sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH); //read high pulse duration of sensor 2

  t2 = duration2 / 200; //convert duration of sensor 2 into total time value in microseconds
  distance2 = a2 * t2 + b2; //use calibration coefficients of sensor 2 to calculate distance in mm

  //Kalman filter correction 1
  K = P1 / (P1 + R1); //kalman gain
  yk1 = yk + K * (distance1 - yk); //updated state estimate using first sensor
  P = (1 - K) * P1; //updated state covariance of first sensor

  //Kalman filter correction 2
  K = P / (P + R2); //kalman gain
  yk2 = yk1 + K * (distance2 - yk1); //updated state estimate using second sensor (distance to output)
  P2 = (1 - K) * P; //updated state covariance using second sensor (variance to output)

  //send and receive sound waves from ultrasonic sensor 1
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH); //read high pulse duration of sensor 1

  t3 = duration3 / 200; //convert duration of sensor 1 into total time value in microseconds
  distance3 = a3 * t3 + b3; //use calibration coefficients of sensor 1 to calculate distance in mm

  //sensor fusion
  //Kalman filter prediction stage
  t_yk = t_yk2; //use the first distance as prediction
  t_P1 = t_P + t_Q; //predicted state covariance

  //send and receive sound waves from ultrasonic sensor 4
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH); //read high pulse duration of sensor 4

  t4 = duration4 / 200; //convert duration of sensor 4 into total time value in microseconds
  distance4 = a4 * t4 + b4; //use calibration coefficients of sensor 4 to calculate distance in mm

  //Kalman filter correction 3
  t_K = t_P1 / (t_P1 + t_R1); //kalman gain
  t_yk1 = t_yk + t_K * (distance3 - t_yk); //updated state estimate using first sensor
  t_P = (1 - t_K) * t_P1; //updated state covariance of first sensor

  //Kalman filter correction 4
//  t_K = t_P / (t_P + t_R2); //kalman gain
//  t_yk2 = t_yk1 + t_K * (distance4 - t_yk1); //updated state estimate using second sensor (distance to output)
//  t_P2 = (1 - t_K) * t_P; //updated state covariance using second sensor (variance to output)

  // PID Control for acc
  L = yk2;
  ek = L - L0; // If the vehicle run reverely, change it to: ek = L0 - L
  uk = uk_1 + k1 * ek + k2 * ek_1 + k3 * ek_2;

  uk = constrain(uk, 1445, 1535);

  // PID Control for lka
  D = t_yk1;
  t_ek = D0 - D;
  t_uk = t_uk_1 + t_k1 * t_ek + t_k2 * t_ek_1 + t_k3 * t_ek_2;

  t_uk = constrain(t_uk, 75, 105);

  if (ek / ek_1 < 0) {
    uk = 1500;
    delay(500);
  }

//  if (t_ek / t_ek_1 < 0) {
//    t_uk = 90;
//    delay(100);
//  }

  ek_1 = ek;
  ek_2 = ek_1;
  uk_1 = uk;

  t_ek_1 = t_ek;
  t_ek_2 = t_ek_1;
  t_uk_1 = t_uk;
  
  // Call the setVehicle function to set the vehicle steering and velocity(speed) values
  
  if (uk > 1500)
  {
    check = 1; //forward
  }
  else if (uk < 1500) 
  {
    check = 0; //backward
  }
  
  if (check == 1 && (ek - ek_1 > 150) && ek < 1500) //flying condition
  {
    setVehicleHR(steering, 1500);
    delay(50);
    setVehicleHR(steering, 1400);
    delay(500);
    setVehicleHR(steering, 1500);
    delay(50);
  }
  else if (check == 1 && ek > 1500) //far away condition (need to be tested for neutral case)
  {
    setVehicleHR(t_uk, uk);
    delay(100);   
    setVehicleHR(steering, 1500);
    delay(100); 
  }
  else if (uk == 1500 && ek > 1000)
  {
    setVehicleHR(t_uk, uk);
    delay(400);
    setVehicleHR(steering, 1500);
    delay(100);
  }
  else //(close case)
  {
      setVehicleHR(steering, uk);
      delay(50);
      setVehicleHR(steering, 1500);
      delay(50);
  }  


  //Serial.println(yk2);
  //Serial.println(yk1);
//  Serial.print(", ");
//  Serial.print(ek);
//  Serial.print(", ");
//  Serial.print(uk);
//  Serial.print(", ");
    Serial.print(t_yk1);
  Serial.print(", ");
  Serial.print(t_ek);
  Serial.print(", ");
  Serial.println(t_uk);

  //delay(100);

}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicleHR(int s, int v)
{
  s = min(max(0, s), 180); //saturate steering command
  v = min(max(1400, v), 1600); //saturate throttle/velocity command
  ssm.write(s); //write steering value to steering servo
  esc.writeMicroseconds(v); //write throttle/velocity value to the ESC unit
}

//***************** Do not change above part *****************//
