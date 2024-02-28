// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftLift = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax rightLift = new CANSparkMax(8, MotorType.kBrushless);

  private final RelativeEncoder leftLiftEncoder = leftLift.getEncoder();
  private final RelativeEncoder rightLiftEncoder = rightLift.getEncoder();

  public Climb() {}

  public void setLeftClimb(double speed){
    leftLift.set(speed);
  }
  public void setRightClimb(double speed){
    rightLift.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
