// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(12);
  private final CANSparkMax frontRight = new CANSparkMax(3,  MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(4,  MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(1,  MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(2,  MotorType.kBrushless);

  private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
  private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = backRight.getEncoder();
  private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  final double kRevToFeet = 1188 * Math.PI / 25296;

  public Drivetrain() {
    backRight.follow(frontRight);
    backLeft.follow(frontLeft);
    frontLeft.setInverted(true);
    drive.setMaxOutput(0.2);
    gyro.setYaw(0);
    frontRightEncoder.setPositionConversionFactor(kRevToFeet);
    frontLeftEncoder.setPositionConversionFactor(kRevToFeet);
    backRightEncoder.setPositionConversionFactor(kRevToFeet);
    backLeftEncoder.setPositionConversionFactor(kRevToFeet);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
