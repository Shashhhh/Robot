// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <AHRS.h>
#include <math.h>
#include <frc/AddressableLED.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/SpeedController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Controller.h>
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>

using namespace rev;
#define FEET_PER_COUNT ((6*M_PI/360)/12.0) //Math for encoder
#define RADIANS_PER_DEGREE (M_PI/180) //Math for the gyroscope

AHRS *ahrs; //Intiating gyroscope
float y_pos; //Forward and backward position of robot
float x_pos; //Left and right position of robot
float rightlastvalue; //Last value of the right encoder
float leftlastvalue; //Last value of the left encoder
float angleoffset; //Error of the robot angle
float pitchoffset;
float r_pos;
float x_target = 6;
float y_target = 6; 
int startTime;

CANSparkMax m_intake2(10, CANSparkMaxLowLevel::MotorType::kBrushless);
CANSparkMax winch1(9, CANSparkMaxLowLevel::MotorType::kBrushless);
CANSparkMax winch2(11, CANSparkMaxLowLevel::MotorType::kBrushless);
CANSparkMax m_intake(8, CANSparkMaxLowLevel::MotorType::kBrushless); //iniating the elevator
 // iniating intake
VictorSPX m_elevator(1);
CANSparkMax m_rlead(1, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating right lead drive motor
CANSparkMax m_rfollow(2, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating right follow drive motor
CANSparkMax m_llead(3, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating left lead drive motor
CANSparkMax m_lfollow(4, CANSparkMaxLowLevel::MotorType::kBrushless); //intiating left follow drive motor
frc::SpeedControllerGroup m_left(m_llead, m_lfollow); //binding the left side together
frc::SpeedControllerGroup m_right(m_rlead, m_rfollow); //binding the right side together
frc::Joystick m_drive1(0); //Joystick 1
frc::Joystick m_drive2(1); //Joystick 2
frc::Joystick m_drive3(2); //Joystick 3
frc::DigitalInput m_isensor(0);
frc::Timer m_timer;
SparkMaxRelativeEncoder m_rencoder = m_rfollow.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
SparkMaxRelativeEncoder m_lencoder = m_lfollow.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
/*
  static constexpr int kLength = 60;

  // PWM port 9
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;
*/

void Robot::RobotInit() {
 m_chooser.SetDefaultOption("kAutoNameDefault", kAutoNameDefault);
  m_chooser.AddOption("kAutoNameCustom", kAutoNameCustom);
  
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  /*
  rightlastvalue = 0;

    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

    for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(255, 0, 0);
}

m_led.SetData(m_ledBuffer);
*/





frc::CameraServer::StartAutomaticCapture().SetResolution(320, 240);
 
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  //  m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  m_timer.Reset();
  m_timer.Start();

 
 
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    
 } else {
    // Default Auto goes here
  }
}


void Robot::AutonomousPeriodic() {
  
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
   
// m_rlead.RestoreFactoryDefaults();
}
else
{  

    // Default eAuto goes here
/*
// One Ball Auton Low
if (m_timer.Get() < 1_s)
    {
      m_elevator.Set(VictorSPXControlMode::PercentOutput, .75);
    }
else if (m_timer.Get() < 6_s)
{
   m_elevator.Set(VictorSPXControlMode::PercentOutput, 0);
      m_left.Set(-.2);
      m_right.Set(.2);
}
else {
  m_left.Set(0);
  m_right.Set(0);
}

*/
/*
  
  //Two Ball Auton Low
if (m_timer.Get() < 1_s)
    {
      m_elevator.Set(VictorSPXControlMode::PercentOutput, .75);
     
    } else if (m_timer.Get() < 4_s)
    {
      m_elevator.Set(VictorSPXControlMode::PercentOutput, 0);
      m_left.Set(-.2);
      m_right.Set(.2);
      m_intake.Set(1);
      m_intake2.Set(1);
    } else if (m_timer.Get() < 7_s)
    {
      
      m_left.Set(.202);
      m_right.Set(-.202);
      
    } else if (m_timer.Get() < 7.5_s)
    {
      m_left.Set(0);
      m_right.Set(0);
     
    }else if (m_timer.Get() < 11_s)
    {
m_elevator.Set(VictorSPXControlMode::PercentOutput, .75);
 
    }
    else if (m_timer.Get() < 15.5_s)
    {
    m_left.Set(-.2);
    m_right.Set(.2);
    m_intake.Set(0);
    m_intake2.Set(0);
    m_elevator.Set(VictorSPXControlMode::PercentOutput, 0);
    } else
    {
      m_left.Set(0);
      m_right.Set(0);
    }
*/

// three ball auton
if (m_timer.Get() < .5_s)
m_elevator.Set(VictorSPXControlMode::PercentOutput, .75);
else if (m_timer.Get() < 2.5_s)
{
m_elevator.Set(VictorSPXControlMode::PercentOutput, 0);  
m_left.Set(-.3); //drives forward
m_right.Set(.3);
m_intake.Set(1); // run intake
m_intake2.Set(1);
} else if (m_timer.Get() < 4_s)
{
  m_left.Set(.33);
  m_right.Set(-.1); //turn to back left

} else if (m_timer.Get() < 7.5_s){
m_left.Set(-.3);
m_right.Set(.3); //drives forward
m_elevator.Set(VictorSPXControlMode::PercentOutput, .15); 
} else if (m_timer.Get() < 9.5_s)
{
  m_left.Set(.15); //turn to back right
  m_right.Set(-.2);
} else if (m_timer.Get() < 11.5_s)
{
  m_left.Set(.2); //drive backwards
  m_right.Set(-.26);
} else if (m_timer.Get() < 12.5_s)
{
  m_elevator.Set(VictorSPXControlMode::PercentOutput, .75); 
  m_left.Set(0);
m_right.Set(0);
}
else
{
m_left.Set(0);
m_right.Set(0);
m_intake.Set(0);
m_intake2.Set(0);
m_elevator.Set(VictorSPXControlMode::PercentOutput, 0); 
}




    float rcurrent = m_rencoder.GetPosition() * FEET_PER_COUNT;
    float lcurrent = m_lencoder.GetPosition() * FEET_PER_COUNT;
    
  
    frc::SmartDashboard::PutNumber("right encoder",rcurrent);
    frc::SmartDashboard::PutNumber("left encoder",lcurrent);

  } 
    // m_rlead.RestoreFactoryDefaults();
    
  
}
  


void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {



float x_axis = m_drive1.GetRawAxis(2)/2.5; 
float y_axis = m_drive1.GetRawAxis(1)/2.5;
float x2_axis = m_drive1.GetRawAxis(2)/4;
float y2_axis = m_drive1.GetRawAxis(1)/4;



if (m_drive1.GetRawButton(1)){
m_left.Set(x2_axis + y2_axis); 
m_right.Set(x2_axis - y2_axis); 

} else if (m_drive1.GetRawButton(7)){
  m_left.Set((-x_axis - y_axis)); 
  m_right.Set((-x_axis + y_axis)); 
} else if (m_drive1.GetRawButton(2)) {
  m_left.Set(-(x_axis + y_axis)); 
m_right.Set(-(x_axis - y_axis)); 
}
else {

m_left.Set(x_axis + y_axis); 
m_right.Set(x_axis - y_axis); 
}


if (m_drive2.GetRawButton(8))
{
winch1.Set(-.5);
winch2.Set(-.5);
}

else
{
  winch1.Set(0);
  winch2.Set(0);
}



//m_left.Set(x_axis + y_axis); 
//m_right.Set(x_axis - y_axis); 

 
 if(m_drive2.GetRawButton(1)) {
  m_elevator.Set(VictorSPXControlMode::PercentOutput, .75);

 } else if (m_drive2.GetRawButton(2)) 
 { m_elevator.Set(VictorSPXControlMode::PercentOutput, -.75);

} else {
  m_elevator.Set (VictorSPXControlMode::PercentOutput, 0);
} 

if(m_drive2.GetRawButton(3)) {
  m_intake.Set(1);
  m_intake2.Set(1);
}
  else if (m_drive2.GetRawButton(4)) {
  m_intake.Set(-1);
  m_intake2.Set(-1);
  
  } else {
  m_intake.Set(0);
  m_intake2.Set(0);
  
} 





}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
