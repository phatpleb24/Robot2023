// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/Joystick.h"
#include "frc/PS4Controller.h"
#include <frc2/command/InstantCommand.h>

#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  Drivetrain& GetDrive() {return m_drive;};

  frc2::Command* GetAutonomousCommand();

  frc2::Command* TankDriveCommand();

  frc2::InstantCommand m_driveHalfSpeed{[this] {m_drive.SetMaxOutput(0.5);}, {}};
  frc2::InstantCommand m_driveFullSpeed{[this] {m_drive.SetMaxOutput(1);}, {}};

 private:
  // The robot's subsystems and commands are defined here...
  //frc::SendableChooser<frc2::Command*> m_chooser;
  Drivetrain m_drive;
  // frc::Joystick m_joystick{0};
  frc::PS4Controller m_ps4{0};
  void ConfigureButtonBindings();
};
