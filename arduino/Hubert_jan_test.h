

#include <Arduino.h>
#include <Servo.h>


//Servos
Servo body;
Servo shoulder;
Servo elbow;
Servo gripper;

//Init position of all servos
const int servo_pins[] = {3, 9, 10, 11};
const int pos_initStart[] = {1700, 2200, 1650, 1600};
const int minimum[] = {560, 750, 550, 550};
const int maximum[] = {2330, 2200, 2400, 2150};

// max angles servos: 1770,1790,1450,1450,1850,1600
int pos_init[] = {1140, 1450, 1100, 1050};
int curr_pos[4];

float min_height = 0.15;
float L2 = 0.315;
float L3 = 0.045;
float L4 = 0.108;
float L5 = 0.005;
float L6 = 0.034;
float L7 = 0.015;
float L8 = 0.088;
float L9 = 0.204;


bool table_avoidence(int theta2, int theta3){
  float t2 = theta2/10 * 3.14159/180;
  float t3 = theta3/10 * 3.14159/180;
  float s2 = sin(t2);
  float c2 = cos(t2);
  float s3 = sin(t3);
  float c3 = cos(t3);
  bool min_height < L2 + L3 - c2*(L8 + c3 L9) + L7*s2 + L9*s2*s3;
  
  return min_height;
}

void servo_move_everything(int new_pos_body, int new_pos_shoulder, int new_pos_elbow, int new_pos_gripper) {
  int max[] = {maximum[0],maximum[1],maximum[2],maximum[3]};
  int new_pos[] = {new_pos_body + minimum[0], new_pos_shoulder + minimum[1],new_pos_elbow + minimum[2],new_pos_gripper + minimum[3]};
  
  int delta = 6;

  //current servo value
  int now[] = {curr_pos[0],curr_pos[1],curr_pos[2],curr_pos[3]};
  int direction[4];
  int distance[4]; //difference between new pos and old pos
  
  int intermediate_positions[4][7];
  
  /* determine interation "diff" from old to new position */
  for(int i = 0; i<4;i++){
    direction[i] = (new_pos[i] - now[i])/abs(new_pos[i] - now[i]); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
    distance[i] = abs(new_pos[i] - now[i]);
    for (int j = 0; j < 7; j++){
      intermediate_positions[i][j] = now[i] + j * distance[i]/6;
    }
  }
  
  // move to position
  int body_step = 1;
  int shoulder_step = 0;
  int elbow_step = 0;
  while (body_step != 7 && sholder_step != 7 && elbow_step != 7) {
    int body_pos = intermediate_positions[0][body_step];
    int shoulder_pos = intermediate_positions[1][shoulder_step];
    int elbow_pos = intermediate_positions[2][elbow_step];
    
    bool no_update = true;
    
    body.writeMicroseconds(body_pos);
    body_step = min(body_step++, 7);
    
    if (table_avoidance(intermediate_positions[1][shoulder_step + 1], elbow_pose)){
      shoulder.writeMicroseconds(intermediate_positions[1][shoulder_step + 1]);
      shoulder_step = min(shoulder_step++, 7);
      shoulder_pos = intermediate_positions[1][shoulder_step];
      no_update = false;
      Serial.printLn("Collision for shoulder");
    }
  
    if (table_avoidance(shoulder_pos, intermediate_positions[2][elbow_step + 1])){
      shoulder.writeMicroseconds(intermediate_positions[2][elbow_step + 1]);
      elbow_step = min(elbow_step++, 7);
      elbow_pos = intermediate_positions[2][elbow_step];
      no_update = false;
      Serial.printLn("Collision for elbow");
    }
    
    if (!no_update && body_step == 7) {
      break;
    }
    delay(20);
  }
}

void setup() {

  Serial.begin(57600); // Starts the serial communication
  
  //Attach each joint servo
  //and write each init position
  body.attach(servo_pins[0]);
  body.writeMicroseconds(pos_initStart[0]);
  shoulder.attach(servo_pins[3]);
  shoulder.writeMicroseconds(pos_initStart[3]);  
  elbow.attach(servo_pins[4]);
  elbow.writeMicroseconds(pos_initStart[4]);
  gripper.attach(servo_pins[5]);
  gripper.writeMicroseconds(pos_initStart[5]);

  //Initilize curr_pos and new_servo_val vectors
  for (int i=0; i<4; i++){
    curr_pos[i] = pos_init[i] + minimum[i];
  }
  delay(2000);
  Serial.println("Type motor angle, body = 1, shoulder = 2, elbow = 3, gripper = 4");  
}

void loop() {
if(Serial.available()){        
        String command = Serial.readStringUntil('\n');            
        int angle[] = {command.substring(0,3).toInt() * 10,command.substring(3, 6).toInt() * 10,command.substring(6, 9).toInt() * 10,command.substring(9, 12).toInt() * 10};
        
        servo_move_everything(angle[0],angle[1],angle[2],angle[3]);        
     }    
}


































#include <Arduino.h>
#include <Servo.h>


//Servos
Servo body;
Servo headPan;
Servo headTilt;
Servo shoulder;
Servo elbow;
Servo gripper;

//Init position of all servos
const int servo_pins[] = {3, 5, 6, 9, 10, 11};
const int minimum[] = {560, 550, 950, 750, 550, 550};
const int maximum[] = {2330, 2340, 2400, 2200, 2400, 2150};
double maxZ = 0.14;
// max angles servos: 1770,1790,1450,1450,1850,1600
int pos_init[] = {900, 950, 1050, 200, 10, 10};
int curr_pos[6];


//Start Position 090020001120

void servo_move_everything(int new_pos_body, int new_pos_shoulder, int new_pos_elbow, int new_pos_gripper) {
  int max[] = {maximum[0],maximum[3],maximum[4],maximum[5]};
  int new_pos[] = {new_pos_body + minimum[0], new_pos_shoulder + minimum[3],new_pos_elbow + minimum[4],new_pos_gripper + minimum[5]};

  int delta = 6;

  //current servo value
  int now[] = {curr_pos[0],curr_pos[3],curr_pos[4],curr_pos[5]};
  int diff[4];
  int steps[4]; //difference between new pos and old pos
  double theta_1 = (now[0] - minimum[0])*3.1415/1800;
  double theta_2 = (now[1] - minimum[3])*3.1415/1800;
  double theta_3 = -(now[2] - minimum[4])*3.1415/1800;
  double X = 0.034*cos(theta_1) + 0.103*sin(theta_1) - 0.088*cos(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2)) - 0.015*cos(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2)) - 0.204*cos(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) - 0.204*cos(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3));
  double Y = 0.034*sin(theta_1) - 0.103*cos(theta_1) - 0.088*sin(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2)) - 0.015*sin(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2)) - 0.204*sin(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) - 0.204*sin(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3));
  double Z = 0.0624*cos(theta_2) - 0.0639*sin(theta_2) - 0.204*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3)) + 0.204*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) + 0.36;

  /* determine interation "diff" from old to new position */
  for(int i = 0; i<4;i++){
  diff[i] = (new_pos[i] - now[i])/abs(new_pos[i] - now[i]); // Should return +1 if NewPwm is bigger than CurrPwm, -1 otherwise.
  steps[i] = abs(new_pos[i] - now[i]);
  }
  delay(10);

  for (int i = 0; i < 4; i++){
    if (new_pos[i] <= max[i] ){
    for (int j = 0; j < steps[i]; j += delta) {
    if((diff[0]==-1 && now[0] > new_pos[0]) || (diff[0] == 1 && now[0] < new_pos[0])){
      now[0] = now[0] + delta*diff[0];
      body.writeMicroseconds(now[0]);
      delay(20);
    }
    if((diff[1]==-1 && now[1] > new_pos[1]) || (diff[1]==1 && now[1] < new_pos[1])){
      theta_2 = (now[1] - minimum[3])*3.1415/1800;
      Z = 0.0624*cos(theta_2) - 0.0639*sin(theta_2) - 0.204*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3)) + 0.204*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) + 0.36;
      if (Z > maxZ || new_pos[1] < now[1]){
      now[1] = now[1] + delta*diff[1];
      shoulder.writeMicroseconds(now[1]);
      delay(20);
      }
    }
    if((diff[2]==-1 && now[2] > new_pos[2]) || (diff[2]==1 && now[2] < new_pos[2])){
      theta_3 = -(now[2] - minimum[4])*3.1415/1800;
      Z = 0.0624*cos(theta_2) - 0.0639*sin(theta_2) - 0.204*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3)) + 0.204*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) + 0.36;
      if (Z > maxZ){
      now[2] = now[2] + delta*diff[2];
      elbow.writeMicroseconds(now[2]);
      delay(20);
      }
    }
    if((diff[3]==-1 && now[3] > new_pos[3]) || (diff[3]==1 && now[3] < new_pos[3])){
      now[3] = now[3] + delta*diff[3];
      gripper.writeMicroseconds(now[3]);
      delay(20);
    }
    }
  }
  }
  curr_pos[0] = now[0];
  curr_pos[3] = now[1];
  curr_pos[4] = now[2];
  curr_pos[5] = now[3];
  delay(10);
  //Position 0 = 090144088120
  theta_1 = (now[0] - minimum[0])*3.1415/1800;
  theta_2 = (now[1] - minimum[3])*3.1415/1800;
  theta_3 = -(now[2] - minimum[4])*3.1415/1800;
  X = 0.034*cos(theta_1) + 0.103*sin(theta_1) - 0.088*cos(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2)) - 0.015*cos(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2)) - 0.204*cos(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) - 0.204*cos(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3));
  Y = 0.034*sin(theta_1) - 0.103*cos(theta_1) - 0.088*sin(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2)) - 0.015*sin(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2)) - 0.204*sin(theta_1)*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) - 0.204*sin(theta_1)*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3));
  Z = 0.0624*cos(theta_2) - 0.0639*sin(theta_2) - 0.204*(0.588*cos(theta_2) + 0.809*sin(theta_2))*(0.999*cos(theta_3) + 0.0349*sin(theta_3)) + 0.204*(0.809*cos(theta_2) - 0.588*sin(theta_2))*(0.0349*cos(theta_3) - 0.999*sin(theta_3)) + 0.36;

  Serial.println(theta_2 * 180/3.1415);
  Serial.println(theta_3 * 180/3.1415);
  Serial.println(Z,4);

}

void setup() {
  //Start Position 090020001120
  // pos_initStart[] = {1700, 1500, 2000, 2200, 1650, 1600};
  // minimum[] = {560, 550, 950, 750, 550, 550};
  Serial.begin(57600); // Starts the serial communication

  //Attach each joint servo
  //and write each init position
  body.attach(servo_pins[0]);
  body.writeMicroseconds(pos_init[0] + minimum[0]);
  headPan.attach(servo_pins[1]);
  headPan.writeMicroseconds(pos_init[1] + minimum[1]);
  headTilt.attach(servo_pins[2]);
  headTilt.writeMicroseconds(pos_init[2] + minimum[2]);
  shoulder.attach(servo_pins[3]);
  shoulder.writeMicroseconds(pos_init[3] + minimum[3]);
  elbow.attach(servo_pins[4]);
  elbow.writeMicroseconds(pos_init[4] + minimum[4]);
  gripper.attach(servo_pins[5]);
  gripper.writeMicroseconds(pos_init[5] + minimum[5]);

  //Initilize curr_pos and new_servo_val vectors
  byte i;
  for (i=0; i<(sizeof(pos_init)/sizeof(int)); i++){
    curr_pos[i] = pos_init[i] + minimum[i];
  }
  delay(2000);
  // in MaxAngles in degress 1770,1790,1450,1450,1850,1600
  Serial.println("Type motor angles");
  Serial.println("");
  Serial.println("max angles:");
// minimum[] =         {560, 550, 950, 750, 550, 550};

}


void loop() {
if(Serial.available()){
        //String command = Serial.readStringUntil('\n');
        //Serial.println(command);
        //int angle[] = {command.substring(0,3).toInt()* 10,command.substring(3, 6).toInt() * 10,command.substring(6, 9).toInt() * 10,command.substring(9, 12).toInt() * 10};
        int theta1 = random(1770); int theta2 = random(1450); int theta3 = random(1850); int theta4 = random(1600);
        int angle[] = {theta1,theta2,theta3,theta4};
        servo_move_everything(angle[0],angle[1],angle[2],angle[3]);
        delay(200);
   }
}