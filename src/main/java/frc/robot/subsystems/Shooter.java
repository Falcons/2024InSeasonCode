// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX leftFlywheel = new TalonFX(6);
  private final TalonFX rightFlywheel = new TalonFX(5);
  public Shooter() {
    leftFlywheel.setInverted(false);
    rightFlywheel.setInverted(true);
  }

  public void fire(double leftSpeed, double rightSpeed) {
    rightFlywheel.set(rightSpeed);
    leftFlywheel.set(leftSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
