// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "RobotContainer.h"
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
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <wpi/fs.h>
#include <frc2/command/button/JoystickButton.h>
#include <photonlib/PhotonUtils.h>
#include <numbers>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "commands/PlacementSequence.h"
#include "commands/Balance.h"
#include <frc/Timer.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

}

void RobotContainer::ConfigureButtonBindings() {
   m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        if(m_joystick.GetLeftStickButtonPressed())
        {
          if(m_drive.table->GetNumber("camMode", 0) == 0)
          {
            m_drive.table->PutNumber("camMode", 1);
          }
          else
          {
            m_drive.table->PutNumber("camMode", 0);
          }
        }
        if(m_joystick.GetRightStickButton())
        {
          m_drive.slewRateFlag = !m_drive.slewRateFlag;
        }
        double x = -m_joystick.GetRawAxis(1);
        double z = 0.0;
        if(m_joystick.GetAButton())
        {
          //photonlib::PhotonPipelineResult result = m_drive.camera.GetLatestResult();
          if(m_drive.table->GetNumber("tv", 0.0))
          {
            if (m_drive.table->GetNumber("tx", 0.0) < -3.0)
            {
              z = 0.37;
            }
            if (m_drive.table->GetNumber("tx", 0.0) > 3.0)
            {
              z = -0.37;
            }
            //z = controller.Calculate((result.GetBestTarget().GetYaw()), 0);
          }
        }
        else 
        {
          z = (m_joystick.GetRawAxis(2) - m_joystick.GetRawAxis(3)) / 2.0;
        }
        m_drive.ArcadeDrive(x, z);
        frc::SmartDashboard::PutNumber("Joystick x", x);
        frc::SmartDashboard::PutNumber("Joystick z", z);
      },
      {&m_drive}));

  m_arm.SetDefaultCommand(frc2::RunCommand
  {
    [this]
    {
      double x = m_joystick2.GetRawAxis(5);
      if(x > 0.1) m_arm.armStall = false;
      if (std::abs(x)>0.1){
        m_arm.armBack = 0;
      }
      m_arm.moveArm(3_V * x);
      if (m_joystick2.GetBButtonPressed()){
        m_arm.armStall = false;
        m_arm.armBack = 1;
      }
      if (m_arm.armBack == 1){
        m_arm.moveArm(0.5_V);
      }
      if(m_joystick2.GetLeftBumperPressed())
      {
        if (m_arm.intakeState != 1){
          m_arm.intakeState = 1;
        }
        else {
          m_arm.intakeState = 0;
        }
      }
      else if(m_joystick2.GetRightBumperPressed())
      {
        if (m_arm.intakeState != -1){
          m_arm.intakeState = -1;
        }
        else {
          m_arm.intakeState = 0;
        }
      } 
      if (m_arm.intakeState==1){
        m_arm.moveIntake(7.5_V);
      }
      else if (m_arm.intakeState==-1){
        m_arm.moveIntake(-9_V);
      }
      else if (m_arm.intakeState==0){
        m_arm.moveIntake(0_V);
      }
      /*if(m_joystick2.GetLeftBumper()){
        m_arm.moveIntake(12_V);
      }
      else if (m_joystick2.GetRightBumper()){
        m_arm.moveIntake(-9_V);
      }
      else {
        m_arm.moveIntake(0_V);
      }*/
    },
    {&m_arm}
  });
  Balance* balanceCMD = new Balance(&m_drive);
  m_joystick.B().WhileTrue(balanceCMD);
}


frc2::CommandPtr RobotContainer::Autonomous(std::string file) {
  frc::Trajectory pathWeaverTraj;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / file;
  pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  
  m_drive.resetOdometry(pathWeaverTraj.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    pathWeaverTraj,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
    frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{.5, 0, 0},
    frc2::PIDController{.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };
  PlacementSequence placeCMD = PlacementSequence(&m_arm);
  return std::move(ramseteCommand).ToPtr();//.AndThen(std::move(ramseteCommand).ToPtr()).AndThen([this] {m_drive.tankDriveVolts(0_V,0_V);}, {&m_drive});
}

frc2::CommandPtr RobotContainer::AprilTagTrajectory() {
  
  printf("KelvinDU1 %.03f\n", frc::Timer::GetFPGATimestamp().value());
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint
  {
    frc::SimpleMotorFeedforward<units::meters>
    {
      DriveConstants::kS, DriveConstants::kV, DriveConstants::kA
    },
    frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth), 2_V
  };
  
  frc::TrajectoryConfig config(0.5_mps, 0.4_mps_sq);
  config.SetKinematics(frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth));
  config.AddConstraint(autoVoltageConstraint);
  frc::Pose2d initialPose = m_drive.getPose();
  frc::Pose2d finalPose;
  
  
  printf("KelvinDU2 %.03f\n", frc::Timer::GetFPGATimestamp().value());
  if(m_drive.table->GetNumber("tv", 0.0))
  {
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      vision::cameraHeight, vision::targetHeight, vision::cameraPitch, units::degree_t{m_drive.table->GetNumber("ty", 0.0)}
    );
    frc::Translation2d translation(range, frc::Rotation2d{units::degree_t{-m_drive.table->GetNumber("tx", 0.0)}});
    frc::Transform2d transform(translation, frc::Rotation2d{0_deg});
    finalPose = initialPose.TransformBy(transform);
    
  }
  else finalPose = frc::Pose2d{2_m,4_m,0_deg};
  
  printf("KelvinDU3 %.03f\n", frc::Timer::GetFPGATimestamp().value());

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    initialPose,
    {/*frc::Translation2d{2_m,0_m}, frc::Translation2d{1_m,1_m}*/},
    finalPose,
    config
  );
  

  printf("KelvinDU4 %.03f\n", frc::Timer::GetFPGATimestamp().value());

  frc2::RamseteCommand ramseteCommand
  {
    trajectory,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
    frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{.5, 0, 0},
    frc2::PIDController{.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };
printf("KelvinDU5\n");
  return frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.tankDriveVolts(0_V, 0_V); }, {})).ToPtr();
      //lmoo
}

frc2::CommandPtr RobotContainer::midChargeCMD()
{
  frc::Trajectory pathWeaverTraj;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / "moveToChargeMidpath.wpilib.json";
  pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  m_drive.resetOdometry(pathWeaverTraj.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    pathWeaverTraj,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{DriveConstants::kS, DriveConstants::kV, DriveConstants::kA},
    frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{0.5, 0, 0},
    frc2::PIDController{0.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };
  Balance balanceCMD = Balance(&m_drive);
  PlacementSequence placeCMD = PlacementSequence(&m_arm);
  //return std::move(placeCMD).ToPtr().AndThen(std::move(ramseteCommand).ToPtr()).AndThen(std::move(balanceCMD).ToPtr());
  return std::move(ramseteCommand).ToPtr();
}