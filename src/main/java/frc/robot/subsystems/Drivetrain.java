// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.RobotController;

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

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontLeftEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontLeftEncoder.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRight.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(frontRightEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(frontRightEncoder.getVelocity(), MetersPerSecond));
              },
              this));

  public Drivetrain() {
    backRight.follow(frontRight);
    backLeft.follow(frontLeft);

    frontRight.setInverted(true);
    drive.setMaxOutput(0.2);


    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    gyro.setYaw(0);
    
    frontRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    frontLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backRightEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);
    backLeftEncoder.setPositionConversionFactor(DriveConstants.RevToMetre);

    frontLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    frontRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backRightEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);
    backLeftEncoder.setVelocityConversionFactor(DriveConstants.RPMToMetresPerSecond);

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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
