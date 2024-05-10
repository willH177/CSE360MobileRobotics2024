/**
 * BICOPTER with altitude control
 * This code runs a bicopter with altitude control using the feedback from a barometer.
 * For this example, your robot needs a barometer sensor.
 */

#include "BlimpSwarm.h"
#include "robot/RobotFactory.h"
#include "state/nicla/NiclaConfig.h"
#include "comm/BaseCommunicator.h"
#include "comm/LLC_ESPNow.h"
#include "util/Print.h"
#include "sense/SensorSuite.h"
#include <Arduino.h>
#include <ESP32Servo.h>


// Robot
Robot* myRobot = nullptr;


nicla_t terms; 

// Communication
BaseCommunicator* baseComm = nullptr;

// Control input from base station
ControlInput behave;
ControlInput cmd;
ReceivedData rcv;

// Data storage for the sensors
float senses[myRobot->MAX_SENSORS];

bool updateParams = true;
const int TIME_STEP_MICRO = 4000;

int dt = 1000;
unsigned long clockTime;
unsigned long printTime;

int niclaOffset = 11;


void setup() {
    Serial.begin(115200);
    Serial.println("Start!");
    clockTime = micros();
    printTime = micros();
    // init communication
    baseComm = new BaseCommunicator(new LLC_ESPNow());
    baseComm->setMainBaseStation();

    // init robot with new parameters
    myRobot = RobotFactory::createRobot("FullBicopter");
    
    paramUpdate();

    // updates the ground altitude for the ground feedback
    // TODO: make some way to access the actual ground height from robot
    int numSenses = myRobot->sense(senses);

}

float nicla_yaw = 0;
float z_estimator = 0;
float g_yaw = 0;
const float G_YAW_INCREMENT = 0.05; // Increment g_yaw by 0.5 radians every sec
unsigned long g_yawUpdateTime = 0;
const unsigned long G_YAW_UPDATE_INTERVAL = 100000;

void loop() {
  // Retrieves cmd.params from ground station and checks flags
  recieveCommands();

  // Get sensor values
  int numSenses = myRobot->sense(senses);

  // send values to ground station
  rcv.flag = 1;
  rcv.values[0] = senses[1];  //height
  rcv.values[1] = senses[5];  //yaw
  rcv.values[2] = senses[niclaOffset + 1];  //nicla x
  rcv.values[3] = senses[niclaOffset + 2];  //nicla y
  rcv.values[4] = senses[niclaOffset + 3];  //nicla w
  rcv.values[5] = senses[niclaOffset + 4];  //nicla h
  bool sent = baseComm->sendMeasurements(&rcv);

  // print sensor values every second
  // senses => [temperature, altitude, veloctity_in_altitude, roll, pitch, yaw, rollrate, pitchrate, yawrate, null, battery, nicla_flag, nicla_x, nicla_y, nicla_w, nicla_h, nicla...]
  if (micros() - printTime > 515106){
    
    // for (int i = 0; i < numSenses-1; i++){
    //   Serial.print(senses[i]);
    //   Serial.print(",");
    // }
    // Serial.println(senses[numSenses-1]);
    printTime = micros();
  }


  float fx = 0;
  float flag = 0;
  // Nicla controller (when the incomming flag = 2)
  if (cmd.params[0] == 2) {
    int nicla_flag = (int)senses[niclaOffset + 0];
    if (nicla_flag != 0) {// checks if new yaw occurs
        fx = 0.2;
        flag = 1;
        float _yaw = senses[5];  
        float _height = senses[1];  
        float tracking_x = (float)senses[niclaOffset + 1];
        float tracking_y = (float)senses[niclaOffset + 2];
        
        float x_cal = tracking_x / terms.n_max_x; // normalizes the pixles into a value between [0,1]
        float des_yaw = ((x_cal - 0.5)) * terms.x_strength; // normalizes the normal to between [-.5, .5] to act as an offset for yaw
        nicla_yaw = _yaw + des_yaw; // add the offset in yaw to the current yaw for movement.
        float y_cal = tracking_y / terms.n_max_y;
        if ( abs(x_cal - 0.5) < .16 && terms.y_strength != 0) { // makes sure yaw is in center before making height adjustments
            z_estimator =  ( _height + terms.y_strength * (y_cal - terms.y_thresh)) ; // height control doenst work well when not 0 bouyant
        }
    } 

    behave.params[0] = cmd.params[0]; // flag
    behave.params[1] = fx; // fx ('meters'/second)
    behave.params[2] = cmd.params[2]; // fz (meters)
    behave.params[3] = 0; // tx (radians/second)
    
    if(flag != 0){
      behave.params[4] = nicla_yaw; // tz (radians)
    }
    // else{ //not tracking target
    //   g_yaw += 0.005;
    //   if(g_yaw >= 3.14159){
    //      g_yaw = -3.14159;
    //   }
    //   behave.params[4] = g_yaw; // tz (radians)

    // }
    else{
      if (micros() - g_yawUpdateTime > G_YAW_UPDATE_INTERVAL) {//check if 1 sec has passed
        g_yaw += G_YAW_INCREMENT;
        if (g_yaw >= 3.14) {
          g_yaw = -3.14;
        }
        g_yawUpdateTime = micros(); // Reset the time for the next update
      }
      behave.params[4] = g_yaw; // tz (radians)
      Serial.println(g_yaw);
    }
    

  } else { // direct control with joystick if 'flag' is not 2
    z_estimator = cmd.params[2];
    nicla_yaw = cmd.params[4]; // autoset for when switch occurs
    behave.params[0] = cmd.params[0]; //flag
    behave.params[1] = cmd.params[1]; //fx
    behave.params[2] = cmd.params[2]; //fz
    behave.params[3] = cmd.params[3]; //tx
    behave.params[4] = cmd.params[4]; //tz
  }

  // Send command to the actuators
  myRobot->control(senses, behave.params, 5);

  // makes the clock rate of the loop consistant.
  fixClockRate();
}

void recieveCommands(){
  if (baseComm->isNewMsgCmd()){
    // New command received
    cmd = baseComm->receiveMsgCmd();
    if (int(cmd.params[11]) == 1 && updateParams){
      paramUpdate();
      updateParams = false;
    } else {
      updateParams = true;
    }
    // Print command
    // Serial.print("Cmd arrived: ");
    // printControlInput(cmd);
  }
}

void paramUpdate(){
    NiclaConfig::getInstance()->loadConfiguration();
    const nicla_t& config = NiclaConfig::getInstance()->getConfiguration();
    terms = config; // Copy configuration data
    myRobot->getPreferences();
    baseComm->setMainBaseStation();

}

void fixClockRate() {

  dt = (int)(micros()-clockTime);
  while (TIME_STEP_MICRO - dt > 0){
    dt = (int)(micros()-clockTime);
  }
  clockTime = micros();
}
