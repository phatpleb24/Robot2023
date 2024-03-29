// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DataLogManager.h>
#include <units/length.h>

void Robot::RobotInit()
{
  SetNetworkTablesFlushEnabled(true);
  
  frc::DataLogManager::Start();

  
  chooser.SetDefaultOption("Place Only", place.get());
  chooser.AddOption("Left", leftCMD.get());
  chooser.AddOption("Right", rightCMD.get());
  chooser.AddOption("Mid", midCMD.get());
  chooser2.SetDefaultOption("Place", "Place");
  chooser2.AddOption("Left", "Left");
  chooser2.AddOption("Mid", "Mid");
  chooser2.AddOption("Right", "Right");
  //frc::SmartDashboard::PutData("Auto Modes", &chooser);
  frc::SmartDashboard::PutData("Auto Moodes", &chooser2);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  //autoCommandPtr = chooser.GetSelected();
  if(chooser2.GetSelected() == "Left")
  {
    m_autonomousCommand = m_container.Autonomous("lowOutRed.wpilib.json");
  }
  else if(chooser2.GetSelected() == "Mid")
  {
    m_autonomousCommand = m_container.midChargeCMD();
  }
  else if(chooser2.GetSelected() == "Right")
  {
    m_autonomousCommand = m_container.Autonomous("lowOut.wpilib.json");
  }
  else m_autonomousCommand = PlacementSequence(&(m_container.m_arm)).ToPtr();
  /*if(autoCommandPtr != nullptr)
  autoCommandPtr->Schedule();*/
  m_autonomousCommand->Schedule();
}

bool checkPose(frc::Pose2d pose)
{
  return pose.X() >= 0.9_m && pose.X() <= 1.1_m && pose.Y() >= 0.9_m && pose.Y() <= 1.1_m;
}

void Robot::AutonomousPeriodic()
{
  if(m_container.m_drive.table->GetNumber("tv",0.0) && aprilTagFlag)
  {
   // delete m_autonomousCommand;
    if(commandCreator == nullptr)
    {
      autoCommandPtr->Cancel();
      m_container.m_drive.tankDriveVolts(0_V,0_V);
      commandCreator = std::make_unique<std::thread>([this] () {m_pendingCommand = m_container.AprilTagTrajectory();});      
    }
    else
    {
      if(commandCreator->joinable())
      {
        commandCreator->join();
        //commandCreator = nullptr;
        m_autonomousCommand = std::move(m_pendingCommand);
        m_autonomousCommand->Schedule();
      }
    
    }
  }
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
  if(autoCommandPtr != nullptr)
    autoCommandPtr->Cancel();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
