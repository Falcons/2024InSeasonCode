// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.pigeonID);
  private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRightID,  MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeftID,  MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(DriveConstants.backRightID,  MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeftID,  MotorType.kBrushless);
  
  private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
  private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = backRight.getEncoder();
  private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  private final DifferentialDriveOdometry odometry;

  public Drivetrain() {
    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontRight.setInverted(true);
    drive.setMaxOutput(0.2);

    
    //convert to m and m/s
    frontRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    frontLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);

    frontRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    
    resetEncoders();

    odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getVelocity(), frontRightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders() {
    frontRightEncoder.setPosition(0);
    frontLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return frontLeftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return frontRightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }
}
