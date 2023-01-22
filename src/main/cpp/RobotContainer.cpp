// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/TeleopArcadeDrive.h"
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <Constants.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <frc2/command/RunCommand.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // m_drive.ArcadeDrive(-m_ps4.GetLeftY()/2.0,
        //                     m_ps4.GetLeftX()/2.0);
        m_drive.ArcadeDrive(-m_ps4.GetLeftY(),
                            m_ps4.GetRawAxis(2), m_ps4.GetRawAxis(3));
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  frc2::JoystickButton(&m_ps4, 1).OnTrue(&m_driveHalfSpeed).OnFalse(&m_driveFullSpeed);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint
  {
    frc::SimpleMotorFeedforward<units::meters>
    {
      DriveConstants::kS, DriveConstants::kV, DriveConstants::kA
    },
    frc::DifferentialDriveKinematics(DriveConstants::trackWidth), 2_V
  };
  
  frc::TrajectoryConfig config(0.5_mps, 0.4_mps_sq);
  config.SetKinematics(frc::DifferentialDriveKinematics(DriveConstants::trackWidth));
  config.AddConstraint(autoVoltageConstraint);
  // frc::Pose2d currentPose{m_drive.getPose()};
  // auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   currentPose,
  //   {m_drive.alterTrans(currentPose, 1_m, 0_m)},
  //   m_drive.alterPos(currentPose, 10_m, 0_m, 0_deg),
  //   config
  // );

  frc::Trajectory pathWeaverTraj;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / "path2.wpilib.json";
  pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  
  m_drive.resetOdometry(pathWeaverTraj.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    pathWeaverTraj,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
    frc::DifferentialDriveKinematics(DriveConstants::trackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{1.5, 0, 0},
    frc2::PIDController{1.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };

  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.tankDriveVolts(0_V, 0_V); }, {}));
      //lmoo
}

frc2::Command* RobotContainer::TankDriveCommand()
{
  return new frc2::InstantCommand([this]{m_drive.tankDriveVolts(2_V,2_V);},{});
}