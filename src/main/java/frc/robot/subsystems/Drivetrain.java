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

  final double kRevToFeet = 1188 * Math.PI / 25296;

  public Drivetrain() {
    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontRight.setInverted(true);
    drive.setMaxOutput(0.2);


    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    gyro.setYaw(0);
    
    frontRightEncoder.setPositionConversionFactor(kRevToFeet);
    frontLeftEncoder.setPositionConversionFactor(kRevToFeet);
    backRightEncoder.setPositionConversionFactor(kRevToFeet);
    backLeftEncoder.setPositionConversionFactor(kRevToFeet);

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

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition(), pose);
  }
}
