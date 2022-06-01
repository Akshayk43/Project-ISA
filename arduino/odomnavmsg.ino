#include <PID_v1.h>
//#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#define Pin_left 2//External interrupt 0, left wheel
#define Pin_right 3//External interrupt 1, right wheel

#define max_linear 20//Maximum linear speed cm/sec
#define max_turn_line 18//Maximum turning line speed
//#define max_angular 1.45
#define max_linear_pwd 255

#define hole_number 2//Number of holes in the code wheel
#define diameter 18.535//Wheel diameter in cm
#define diamete_ratio 1.12167//The ratio coefficient of the wheel diameter of the left wheel relative to the right wheel, to the left, to decrease, and to the right to increase
#define center_speed 220//The initial value of PWM power of the car motor
#define gear_ratio 75//Speed ​​ratio
#define car_width 27//Car width
#define car_length 27//Car length

#define E_left 5//The left-wheel motor enable port of the L298P DC motor drive board is connected to the digital interface 5
#define M_left 4//The left-wheel motor steering port of the L298P DC motor drive board is connected to the digital interface 4
#define E_right 6//Connect the enable port of the right wheel motor of the trolley to the digital interface 6
#define M_right 7//Connect the steering port of the right wheel motor of the trolley to the digital interface 7


int val_right_count_target = 0;//The right wheel code wheel of the trolley counts PID adjustment target value per second, according to this value PID val_rigth;
int val_right = 0;//PWM power value of the right wheel motor of the car
int val_left_count_target = 0;//The left wheel code wheel of the trolley counts the PID adjustment target value per second, according to this value PID val_left;
int val_left = 0;//PWM power value of the left-wheel motor. With the left wheel as the base speed, PID adjusts the speed of the right wheel
int count_left = 0;//Left wheel encoder code disc pulse count value; used for PID adjustment
int count_right = 0;//Pulse count value of right wheel encoder code disc; used for PID adjustment
/
char run_direction ='f';//f: forward; b: backward; s: stop
int linear = 0;//15;//cm/second linear velocity
int angular = 0;//1;//Angular velocity, angular.z of ros
///The turning radius must be greater than half the width of the trolley, that is, the linear/angular must be greater than 13.5, that is, the minimum turning radius is 13.5
/
unsigned long left_old_time = 0, right_old_time = 0;//time stamp
unsigned long time1 = 0, time2 = 0;//time stamp

ros
ros::NodeHandle nh;
//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
//char base_link[] = "/base_link";
//char odom[] = "/odom";
//nav_msgs::Odometry odom1;
void motor_cb(const geometry_msgs::Twist& vel)
{
  linear = vel.linear.x * 100;//The unit in ROS is m/s; here it is converted to cm
  angular = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", motor_cb);

//PID
double left_Setpoint, left_Input, left_Output, left_setpoint;
double left_kp = 1, left_ki = 0.005, left_kd = 0.0001;//kp = 0.040,ki = 0.0005,kd =0.0011;
PID left_PID(&left_Input, &left_Output, &left_Setpoint, left_kp, left_ki, left_kd, DIRECT);

double right_Setpoint, right_Input, right_Output, right_setpoint;
double right_kp = 0.8, right_ki = 0.005, right_kd = 0.0021;//kp = 0.040, ki = 0.0005, kd = 0.0011;
PID right_PID(&right_Input, &right_Output, &right_Setpoint, right_kp, right_ki, right_kd, DIRECT);

void setup() {
 //put your setup code here, to run once:
  Serial.begin(9600);//Start serial communication, the baud rate is 9600b/s
 //reserve 200 bytes for the inputString

  pinMode(M_left, OUTPUT);//Set the control port of the L298P DC motor drive board to output mode
  pinMode(E_left, OUTPUT);
  pinMode(M_right, OUTPUT);
  pinMode(E_right, OUTPUT);

 //Define the interrupt subroutine Code() of external interrupt 0 and 1, and the interrupt trigger is the next edge trigger
 //When the OUT pulse signal of the encoder code disk is interrupted by the falling edge,
 //The execution interrupt subroutine Code() will be automatically called.
  left_old_time = millis();
  right_old_time = millis();
  attachInterrupt(0, Code1, FALLING);//Encoder pulse interrupt function of the left wheel motor of the trolley
  attachInterrupt(1, Code2, FALLING);//Encoder pulse interrupt function of the right wheel motor of the trolley

  nh.initNode();
  nh.subscribe(sub);
 //broadcaster.init(nh);
  left_PID.SetOutputLimits(-254, 254);
  left_PID.SetSampleTime(500);
  left_PID.SetMode(AUTOMATIC);
  left_PID.SetTunings(left_kp, left_ki, left_kd);

  right_PID.SetOutputLimits(-254, 254);
  right_PID.SetSampleTime(500);
  right_PID.SetMode(AUTOMATIC);
  right_PID.SetTunings(right_kp, right_ki, right_kd);

}
//Subroutine block
void advance()//advance
{
  digitalWrite(M_left, HIGH);
  analogWrite(E_left, val_left);
  digitalWrite(M_right, HIGH);
  analogWrite(E_right, val_right);
}
void back()//Back
{
  digitalWrite(M_left, LOW);
  analogWrite(E_left, val_left);
  digitalWrite(M_right, LOW);
  analogWrite(E_right, val_right);
}
void Stop()//stop
{
  digitalWrite(E_right, LOW);
  digitalWrite(E_left, LOW);
}
void loop() {

  nh.spinOnce();

 //put your main code here, to run repeatedly:

  if (angular == 0) {//Go straight
    if (linear> 0) {//forward
      Serial.println("Go Forward!\n");
      if (linear> max_linear)
        linear = max_linear;

      float linear_left = linear;//Left inner circle linear velocity
      float linear_right = linear;//Linear speed of right outer circle


      val_right_count_target = linear_right * gear_ratio/(diameter/hole_number);//The number of holes corresponding to the linear velocity of the left inner ring
      val_left_count_target = linear_left * gear_ratio/(diameter * diamete_ratio/hole_number);//The number of holes corresponding to the linear velocity of the right outer ring

      val_right = linear_right * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, left wheel
      val_left = linear_left * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, right

      left_Setpoint = val_left_count_target;
      right_Setpoint = val_right_count_target;

      advance();
      run_direction ='f';
    } else if (linear <0) {//back
      Serial.println("Go Backward!\n");
      linear = abs(linear);
      if (linear> max_linear)
        linear = max_linear;
      float linear_left = linear;//Left inner circle linear velocity
      float linear_right = linear;//Linear speed of right outer circle


      val_right_count_target = linear_right * gear_ratio/(diameter * diamete_ratio/hole_number);//The number of holes corresponding to the linear velocity of the left inner circle
      val_left_count_target = linear_left * gear_ratio/(diameter/hole_number);//The number of holes corresponding to the linear velocity of the right outer ring

      val_right = linear_right * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, left wheel
      val_left = linear_left * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, right wheel

      left_Setpoint = val_left_count_target;
      right_Setpoint = val_right_count_target;

      back();
      run_direction ='b';
    }

  } else if (angular> 0) {//Turn left
    Serial.println("Turn Left!\n");
    if (linear> max_turn_line)//Limit the maximum turning line speed
    {
      angular = angular * max_turn_line/linear;
      linear = max_turn_line;
    } else if (linear == 0) {
      linear = max_turn_line;
    }

    float radius = linear/angular;//Calculate radius

    if (radius <car_width/2)///If the calculated turning radius is less than the minimum radius, set it to the minimum turning radius
      radius = car_width/2;

    float radius_left = radius-car_width/2;//The radius of the left inner circle
    float radius_right = radius + car_width/2;//Right outer circle radius

    float linear_left = radius_left * angular;//Left inner circle linear velocity
    float linear_right = radius_right * angular;//Linear speed of the right outer circle

    if (linear == max_turn_line) {
      linear_left = 255 * (linear_left/linear_right);
      linear_right = 255;
    }


    val_right_count_target = linear_right * gear_ratio/(diameter/hole_number);//The number of holes corresponding to the linear velocity of the left inner ring
    val_left_count_target = linear_left * gear_ratio/(diameter * diamete_ratio/hole_number);//The number of holes corresponding to the linear velocity of the right outer ring

    val_right = linear_right * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, left wheel
    val_left = linear_left * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, right wheel

    left_Setpoint = val_left_count_target;
    right_Setpoint = val_right_count_target;

    run_direction ='f';
    advance();

  } else if (angular <0) {//Turn right
    Serial.println("Turn Right!");
    if (linear> max_turn_line)//Limit the maximum turning line speed
    {
      angular = angular * max_turn_line/linear;
      linear = max_turn_line;

    } else if (linear == 0) {
      linear = max_turn_line;
    }


    float radius = linear/angular;
    if (radius <car_width/2)///If the calculated turning radius is less than the minimum radius, set it to the minimum turning radius
      radius = car_width/2;

    float radius_left = radius + car_width/2;
    float radius_right = radius-car_width/2;

    float linear_left = radius_left * angular;
    float linear_right = radius_right * angular;

    if (linear == max_turn_line) {
      linear_right = 255 * (linear_right/linear_left);
      linear_left = 255;
    }

    val_right_count_target = linear_right * gear_ratio/(diameter/hole_number);//The number of holes corresponding to the linear velocity of the left inner ring
    val_left_count_target = linear_left * gear_ratio/(diameter * diamete_ratio/hole_number);//The number of holes corresponding to the linear velocity of the right outer ring

    val_right = linear_right * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, left wheel
    val_left = linear_left * (max_linear_pwd/max_linear);//PWD value corresponding to the linear velocity calculated according to the wheel diameter parameter, right wheel

    left_Setpoint = val_left_count_target;
    right_Setpoint = val_right_count_target;

    advance();
    run_direction ='f';
  }
  delay(1000);
  val_left_count_target = 0;
  left_Setpoint = 0;
  val_right_count_target = 0;
  right_Setpoint = 0;
  linear = 0;
  angular = 0;
  Stop();
  run_direction ='s';
}

void PID_left() {
  Serial.println("********************************begin PID left");

  left_Input = count_left * 10;
  left_PID.Compute();

  val_left = val_left + left_Output;
  if (val_left> 255)
    val_left = 255;
  if (val_left <0)
    val_left = 0;
  if (run_direction =='f')//According to the PWM power value of the trolley motor that has just been adjusted, correct the forward or backward state of the trolley in time
    advance();
  if (run_direction =='b')
    back();
  Serial.println("********************************end PID Left");
}
void PID_right() {
  Serial.println("********************************begin PID Right");

  right_Input = count_right * 10;
  right_PID.Compute();
  val_right = val_right + right_Output;
  if (val_right> 255)
    val_right = 255;
  if (val_right <0)
    val_right = 0;
  if (run_direction =='f')//According to the PWM power value of the trolley motor that has just been adjusted, correct the forward or backward state of the trolley in time
    advance();
  if (run_direction =='b')
    back();
  Serial.println("********************************end PID Right");
}
