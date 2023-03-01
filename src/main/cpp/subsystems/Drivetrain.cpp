// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <numbers>
#include <units/voltage.h>
#include <photonlib/PhotonUtils.h>
#include <frc/Timer.h>
#include "LimelightHelpers.h"

Drivetrain::Drivetrain() {
  // Implementation of subsystem constructor goes here.
  Init();
}

void Drivetrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  UpdateOdometry();
  m_field.SetRobotPose(m_estimator.GetEstimatedPosition());
  frc::SmartDashboard::PutNumber("Distance", photonlib::PhotonUtils::CalculateDistanceToTarget(vision::cameraHeight, vision::retroHeight, vision::cameraPitch, units::angle::degree_t{table->GetNumber("ty", 0.0)}).value());
  frc::SmartDashboard::PutNumber("Left Velocity", m_leftFrontMotor.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Right Velocity", m_rightFrontMotor.GetSelectedSensorVelocity());
  /*frc::SmartDashboard::PutNumber("Left Front Voltage", m_leftFrontMotor.GetMotorOutputVoltage());
  frc::SmartDashboard::PutNumber("Right Front Voltage", m_rightFrontMotor.GetMotorOutputVoltage());
  frc::SmartDashboard::PutNumber("Left Follower Voltage", m_leftFollowerMotor.GetMotorOutputVoltage());
  frc::SmartDashboard::PutNumber("Right Follower Voltage", m_rightFollowerMotor.GetMotorOutputVoltage());*/
  frc::SmartDashboard::PutNumber("Left Front Temp", m_leftFrontMotor.GetTemperature());
  frc::SmartDashboard::PutNumber("Left Follower Temp", m_leftFollowerMotor.GetTemperature());
  frc::SmartDashboard::PutNumber("Right Front Temp", m_rightFrontMotor.GetTemperature());
  frc::SmartDashboard::PutNumber("Right Follower Temp", m_rightFollowerMotor.GetTemperature());
}

void Drivetrain::ArcadeDrive(double xaxisSpeed, double l1, double r1) {
  diffDrive.ArcadeDrive(xaxisSpeed/2.0, (r1-l1)/2.0);
}

void Drivetrain::ArcadeDrive(double x, double z)
{
  diffDrive.ArcadeDrive(m_rateLimiter.Calculate(x/1.5), z);
}

void Drivetrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
  m_leftMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
  m_rightMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

  m_drivetrainSim.SetInputs(-m_leftMasterSim.GetMotorOutputLeadVoltage() * 1_V, m_rightMasterSim.GetMotorOutputLeadVoltage() * 1_V);

  m_drivetrainSim.Update(20_ms);

  m_leftMasterSim.SetIntegratedSensorRawPosition(DistanceToNativeUnits(-m_drivetrainSim.GetLeftPosition()));
  m_leftMasterSim.SetIntegratedSensorVelocity(VelocityToNativeUnits(-m_drivetrainSim.GetLeftVelocity()));
  m_rightMasterSim.SetIntegratedSensorRawPosition(DistanceToNativeUnits(m_drivetrainSim.GetRightPosition()));
  m_rightMasterSim.SetIntegratedSensorVelocity(VelocityToNativeUnits(m_drivetrainSim.GetRightVelocity()));
  m_pidgeonSim.SetRawHeading(m_drivetrainSim.GetHeading().Degrees().value());
}

void Drivetrain::UpdateOdometry()
{
  //auto c = kWhellRadiusInches * 2 * std::numbers::pi;
  //auto right_distance = (c/kGearRatio) * m_rightFrontMotor.GetSelectedSensorPosition() / kUnitsPerRevolution;
  //auto left_distance = (c/kGearRatio) * m_leftFrontMotor.GetSelectedSensorPosition() / kUnitsPerRevolution;
  //m_odometry.Update(m_imu.GetRotation2d(), left_distance, right_distance);
  //m_odometry.Update(m_imu.GetRotation2d(), NativeUnitsToDistanceMeters(m_leftFrontMotor.GetSelectedSensorPosition()), NativeUnitsToDistanceMeters(m_rightFrontMotor.GetSelectedSensorPosition()));
  m_estimator.Update(m_imu.GetRotation2d(), NativeUnitsToDistanceMeters(m_leftFrontMotor.GetSelectedSensorPosition()), NativeUnitsToDistanceMeters(m_rightFrontMotor.GetSelectedSensorPosition()));
  /*if(table->GetNumber("tv", 0))
  m_estimator.AddVisionMeasurement(frc::Pose2d(units::meter_t{table->GetNumber("x", 0)}, units::meter_t{table->GetNumber("y", 0)}, m_imu.GetRotation2d()), frc::Timer::GetFPGATimestamp() - (table->GetNumber("tl", 0)/1000.0) - (table->GetNumber("cl", 0)/1000.0));*/
}

void Drivetrain::Init()
{
  m_rightFrontMotor.ConfigFactoryDefault();
  m_rightFollowerMotor.ConfigFactoryDefault();
  m_leftFrontMotor.ConfigFactoryDefault();
  m_leftFollowerMotor.ConfigFactoryDefault();

  m_rightFollowerMotor.Follow(m_rightFrontMotor);
  m_leftFollowerMotor.Follow(m_leftFrontMotor);

  m_rightFrontMotor.SetInverted(DriveConstants::kRightDirection);
  m_rightFollowerMotor.SetInverted(DriveConstants::kRightDirection);
  m_leftFrontMotor.SetInverted(DriveConstants::kLeftDirection);
  m_leftFollowerMotor.SetInverted(DriveConstants::kLeftDirection);

  m_rightFrontMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  m_rightFrontMotor.SetSelectedSensorPosition(0);
  m_leftFrontMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  m_leftFrontMotor.SetSelectedSensorPosition(0);


  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("Left Front Motor", &m_leftFrontMotor);
  frc::SmartDashboard::PutData("Left Follower Motor", &m_leftFollowerMotor);
  frc::SmartDashboard::PutData("Right Front Motor", &m_rightFrontMotor);
  frc::SmartDashboard::PutData("Right Follower Motor", &m_rightFollowerMotor);
  frc::SmartDashboard::PutData("Gyro", &m_imu);
  //frc::SmartDashboard::PutData(&diffDrive);
}

int Drivetrain::DistanceToNativeUnits(units::meter_t position)
{
  double wheelRotation = position / (2 * std::numbers::pi * kWhellRadiusInches);
  double motorRotations = wheelRotation * kGearRatio;
  int sensorCounts = (int)(motorRotations * kUnitsPerRevolution);
  return sensorCounts;
}

int Drivetrain::VelocityToNativeUnits(units::meters_per_second_t velocity)
{
  auto wheelRotationPerSecond = velocity / (2 * std::numbers::pi * kWhellRadiusInches);
  auto motorRotationPerSecond = wheelRotationPerSecond * kGearRatio;
  double motorRotationPer100ms = motorRotationPerSecond * 1_s / k100msPerSecond;
  int sensorCountPer100ms = (int)(motorRotationPer100ms * kUnitsPerRevolution);
  return sensorCountPer100ms;
}

units::meter_t Drivetrain::NativeUnitsToDistanceMeters(double sensorCounts)
{
  double motorRotations = (double)sensorCounts/kUnitsPerRevolution;
  double wheelRotations = motorRotations / kGearRatio;
  units::meter_t position = wheelRotations * (2 * std::numbers::pi * kWhellRadiusInches);
  return position;
}

units::meters_per_second_t Drivetrain::NativeUnitstoVelocityMPS(double sensorCounts)
{
  double motorRotationsPer100ms = sensorCounts / kUnitsPerRevolution;
  auto motorRotationPerSecond = motorRotationsPer100ms / (1_s/k100msPerSecond);
  auto wheelRotationPerSecond = motorRotationPerSecond / kGearRatio;
  auto velocity = wheelRotationPerSecond * (2 * std::numbers::pi * kWhellRadiusInches);
  return velocity;
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{
  m_rightFrontMotor.SetSelectedSensorPosition(0);
  m_rightFollowerMotor.SetSelectedSensorPosition(0);
  m_leftFollowerMotor.SetSelectedSensorPosition(0);
  m_leftFrontMotor.SetSelectedSensorPosition(0);
  //m_odometry.ResetPosition(m_imu.GetRotation2d(), 0_m, 0_m, pose);
  m_estimator.ResetPosition(m_imu.GetRotation2d(), 0_m, 0_m, pose);
}

frc::Pose2d Drivetrain::getPose()
{
 // return m_odometry.GetPose();
  return m_estimator.GetEstimatedPosition();
}

frc::DifferentialDriveWheelSpeeds Drivetrain::getWheelSpeed()
{
  return {NativeUnitstoVelocityMPS(m_leftFrontMotor.GetSelectedSensorVelocity()), NativeUnitstoVelocityMPS(m_rightFrontMotor.GetSelectedSensorVelocity())};
}

void Drivetrain::tankDriveVolts(units::volt_t left, units::volt_t right)
{
  m_leftFrontMotor.SetVoltage(left);
  m_rightFrontMotor.SetVoltage(right);
  diffDrive.Feed();
}

void Drivetrain::SetMaxOutput(double x)
{
  diffDrive.SetMaxOutput(x);
}