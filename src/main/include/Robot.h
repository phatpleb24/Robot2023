// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <thread>
#include <optional>
#include <commands/PlacementSequence.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  std::optional<frc2::CommandPtr> m_midCommand;
  std::optional<frc2::CommandPtr> m_pendingCommand;

  frc2::Command* autoCommandPtr;

  frc::SendableChooser<frc2::Command*> chooser;
  RobotContainer m_container;
  frc2::CommandPtr leftCMD = m_container.Autonomous("lowOutRed.wpilib.json");
  frc2::CommandPtr rightCMD = m_container.Autonomous("lowOut.wpilib.json");
  frc2::CommandPtr midCMD = m_container.midChargeCMD();
  frc2::CommandPtr place = PlacementSequence(&m_container.m_arm).ToPtr();


  std::unique_ptr<std::thread> commandCreator;

  bool aprilTagFlag = false;
};
