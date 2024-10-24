#include <MsTimer2.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#define m_2_pulse     355             // 1m 당 pulse 수  확인 해야 함
#define pulse_2_m     1./355.         // pulse 당 m  확인 해야 함
#define vel_2_pulse   m_2_pulse/20
#define No_Calibration_Point 14


geometry_msgs::Twist cmd_vel;   //모터 명령 수신을 위한 변수(수신) 
std_msgs::Int32 encoder_data1;  //모터 엔코터1 값 전달을 위한 변수(송신)
std_msgs::Int32 encoder_data2;
void cmd_vel_callback(const geometry_msgs::Twist& msg);

ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel_steer", cmd_vel_callback);
ros::Publisher encoder_pub1("encoder1", &encoder_data1);
ros::Publisher encoder_pub2("encoder2", &encoder_data2);


struct CalibrationData {
  double X[No_Calibration_Point];
  double Y[No_Calibration_Point];
} cal_data = {
  {0, 60, 100, 200, 300, 400, 500, 600, 700, 800, 900, 960, 1000, 1023},
  {-24, -19.42, -17.2, -12.66, -8.08, -4.53, -0.17, 4.37, 8.53, 13.12, 16.9, 20.75, 22.83, 23.92}
};

double linear_mapping(double x) {
  if (x <= cal_data.X[0]) {
    return cal_data.Y[0];
  } else if (x >= cal_data.X[No_Calibration_Point - 1]) {
    return cal_data.Y[No_Calibration_Point - 1];
  }

  for (int i = 0; i < No_Calibration_Point - 1; i++) {
    if (x >= cal_data.X[i] && x < cal_data.X[i + 1]) {
      double x1 = cal_data.X[i];
      double x2 = cal_data.X[i + 1];
      double y1 = cal_data.Y[i];
      double y2 = cal_data.Y[i + 1];
      return y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
    }
  }
  return 0;
}

// Front Motor Drive
#define MOTOR1_PWM 5
#define MOTOR1_ENA 6
#define MOTOR1_ENB 4

int f_speed = 0 , r_speed = 0;
int front_motor_pwm = 0;
int velocity = 0;
float steer_angle = 0;

void front_motor_control(int motor1_pwm)
{
   if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}


void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
}

#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23

signed long encoder1count = 0;
signed long encoder2count = 0;
signed long encoder1_error = 0;
signed long encoder1_error_d = 0;
signed long encoder1_target = 0;
signed long encoder1_error_old = 0; 
signed long encoder1_error_sum = 0; 


float target_velocity1 = 0.0;

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}

///////////////////////////////////////  Motor PID 제어 /////////////////////////////////////////////
float Kp_motor = 0.7;
float Kd_motor = 3.0;
float Ki_motor = 0.0001;
//float Kp_motor = 0.8;
//float Kd_motor = 4.1;
//float Ki_motor = 0.0001;

void motor_PID_control(void)
{
  encoder1count = (-1)*readEncoder(1);
  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_sum += encoder1_error;
  encoder1_error_d = encoder1_error - encoder1_error_old;
  encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;
  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;
  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  front_motor_pwm = (front_motor_pwm >=  255) ?  255 : front_motor_pwm;
  front_motor_pwm = (front_motor_pwm <= -255) ? -255 : front_motor_pwm;
  if (fabs(encoder1_error) <= 2)
  {
    encoder1_error_sum = 0;
   //여기서 엔코더를 reset 해야 함 아니면 계속 overflow 발생
    clearEncoderCount(1);     
    //reset후에 target 값을 다시 절정해야 함
    encoder1_target = 0;
  }
  else 
  {
    front_motor_control(front_motor_pwm);
  }  
  encoder1_error_old = encoder1_error;
}

///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////
#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define LEFT_STEER_ANGLE  23.92
#define RIGHT_STEER_ANGLE -24
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

float Kp = 8.5;
float Ki = 0;
float Kd = 0; //PID 상수 설정, 실험에 따라 정해야 함 중요!
double Setpoint, Input, Output; //PID 제어 변수
double error, error_old;
double error_s, error_d;
int pwm_output;
int sensorValue = 0;        // value read from the pot
float Steer_Angle_Measure = 0;        // value output to the PWM (analog out)
float NEURAL_ANGLE = 0.01;
float Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int motor_pwm)
{
    if (motor_pwm > 0) // 전진
    {
        digitalWrite(MOTOR3_ENA, LOW);
        digitalWrite(MOTOR3_ENB, HIGH);
        analogWrite(MOTOR3_PWM, motor_pwm);
    }
    else if (motor_pwm < 0) // 후진
    {
        digitalWrite(MOTOR3_ENA, HIGH);
        digitalWrite(MOTOR3_ENB, LOW);
        analogWrite(MOTOR3_PWM, -motor_pwm);
    }
    else // 정지
    {
        digitalWrite(MOTOR3_ENA, LOW);
        digitalWrite(MOTOR3_ENB, LOW);
        analogWrite(MOTOR3_PWM, 0);
    }
}

void PID_Control()
{
  error = Steering_Angle - Steer_Angle_Measure ;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  pwm_output = int(Kp * error + Kd * error_d + Ki * error_s);
  pwm_output = (pwm_output >=  255) ?  255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(error) <= 0.2)  //특정 값 이하면 제어를 멈추어서 조정이 안되도록
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else          steer_motor_control(pwm_output);
  error_old = error;  
}

void steering_control()
{
  if (Steering_Angle >= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle <= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  PID_Control(); 
}

void control_callback() {
  static boolean output = HIGH;
  
  digitalWrite(13, output);
  output = !output;
  
  encoder1_target += target_velocity1 * vel_2_pulse;
  motor_PID_control();
  
  // Read the analog in value:
  sensorValue = analogRead(Steering_Sensor);
  Steer_Angle_Measure = linear_mapping((double)sensorValue);  // Use the linear mapping function
  
  // Limit Nice control range
  if (Steer_Angle_Measure < RIGHT_STEER_ANGLE) {
    Steer_Angle_Measure = RIGHT_STEER_ANGLE;
  } 
  else if (Steer_Angle_Measure > LEFT_STEER_ANGLE) {
    Steer_Angle_Measure = LEFT_STEER_ANGLE;
  }
  Steering_Angle = NEURAL_ANGLE + steer_angle;
  steering_control();  
}


void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  target_velocity1 = (float)msg.linear.x;
  steer_angle = (float)msg.angular.z;
}

void setup() {
  
  error = error_s = error_d = error_old = 0.0;
  pwm_output = 0;
  
  initEncoders();          // initialize encoder
  clearEncoderCount(1);
  clearEncoderCount(2);
 // Led_pin
  pinMode(13, OUTPUT);

 // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM

  // Please write this code(to use serial)
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(encoder_pub1);
  nh.advertise(encoder_pub2);
  
  MsTimer2::set(50, control_callback); // 500ms period
  MsTimer2::start();
}

void loop(){
  encoder_data1.data = encoder1count;
  encoder_data2.data = encoder2count;
  encoder_pub1.publish(&encoder_data1);
  encoder_pub2.publish(&encoder_data2);
  // No Delay = Rosserial Error
  delay(10);
  nh.spinOnce();
}
