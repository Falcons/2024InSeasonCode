// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax wheels = new CANSparkMax(11, MotorType.kBrushless);

  private final RelativeEncoder pivotEncoder = pivot.getEncoder();
  private final RelativeEncoder wheelsEncoder = wheels.getEncoder();

  public Intake() {}

  public void spinWheels(double speed){
    wheels.set(speed);
  }

  public void pivotSpeed(double speed){
    pivot.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
