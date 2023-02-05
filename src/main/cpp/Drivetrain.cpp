// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  const double leftOutput = m_leftPIDController.Calculate(
      m_leftLeader.GetSelectedSensorVelocity(), speeds.left.value());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightLeader.GetSelectedSensorVelocity(), speeds.right.value());

  m_leftLeader.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightLeader.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t{NativeUnitsToDistanceMeters(m_leftLeader.GetSelectedSensorPosition())},
                    units::meter_t{NativeUnitsToDistanceMeters(m_rightLeader.GetSelectedSensorPosition())});
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_odometry.ResetPosition(m_gyro.GetRotation2d(),
                           units::meter_t{NativeUnitsToDistanceMeters(m_leftLeader.GetSelectedSensorPosition())},
                           units::meter_t{NativeUnitsToDistanceMeters(m_rightLeader.GetSelectedSensorPosition())}, pose);
}

frc::Pose2d Drivetrain::GetPose() const {
  return m_odometry.GetPose();
}

double Drivetrain::NativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kEncoderResolution;
    double wheelRotations = motorRotations / kGearRatio;
    double positionMeters = wheelRotations * (2 * std::numbers::pi * kWheelRadius);
    return positionMeters;
  }