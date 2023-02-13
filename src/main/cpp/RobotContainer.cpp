// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "RobotContainer.h"
#include "commands/TeleopArcadeDrive.h"
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include "Constants.h"
#include <units/velocity.h>
#include <units/acceleration.h>
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
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/PhotonUtils.h>
#include <cmath>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        double x = -m_joystick.GetRawAxis(1);
        double z = 0.0;
        if(m_joystick.GetRawButton(1))
        {
          photonlib::PhotonPipelineResult result = m_drive.camera.GetLatestResult();
          if(result.HasTargets())
          {
            if (result.GetBestTarget().GetYaw() < -3.0)
            {
              z = 0.37;
            }
            if (result.GetBestTarget().GetYaw() > 3.0)
            {
              z = -0.37;
            }
            //z = controller.Calculate((result.GetBestTarget().GetYaw()), 0);
          }
        }
        else z = (m_joystick.GetRawAxis(2) - m_joystick.GetRawAxis(3))/2.0;
        m_drive.ArcadeDrive(x, z);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton(&m_joystick, frc::XboxController::Button::kA).OnTrue(&m_driveHalfSpeed).OnFalse(&m_driveFullSpeed);
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
  frc::Pose2d initialPose = frc::Pose2d{0_m, 0_m, 0_deg};
  frc::Pose2d finalPose;

  photonlib::PhotonPipelineResult result = m_drive.camera.GetLatestResult();
  if(result.HasTargets())
  {
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      cameraHeight, targetHeight, cameraPitch, units::angle::degree_t{target.GetPitch()}
    );
   // units::meter_t x = range * std::cos(target.GetYaw() *  std::numbers::pi / 180.0);
    //units::meter_t y = range * std::sin(target.GetYaw() * std::numbers::pi / 180.0);
    frc::Translation2d translation(range, frc::Rotation2d{units::degree_t{-target.GetYaw()}});
    frc::Transform2d transform(translation, frc::Rotation2d{0_deg});
    finalPose = initialPose.TransformBy(transform);
    
    //finalPose = frc::Pose2d{-x, -y, 0_deg};
    //wpi::outs() << std::to_string(range.value());
    //frc::SmartDashboard::PutNumber("Distance", range.value());
  }
  else finalPose = frc::Pose2d{2_m,2_m,0_deg};

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    initialPose,
    {/*frc::Translation2d{2_m,0_m}, frc::Translation2d{1_m,1_m}*/},
    finalPose,
    config
  );
  frc::Trajectory pathWeaverTraj;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / "path2.wpilib.json";
  pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  
  m_drive.resetOdometry(trajectory.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    trajectory,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
    frc::DifferentialDriveKinematics(DriveConstants::trackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{.5, 0, 0},
    frc2::PIDController{.5, 0, 0},
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