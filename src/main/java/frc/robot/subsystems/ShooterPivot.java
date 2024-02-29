// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(7,  MotorType.kBrushless);

  private final RelativeEncoder pivotEncoder = pivot.getEncoder();
  public ShooterPivot() {}

  public void setSpeed(double speed) {
    pivot.set(speed);
  }

  public void stopShooterPivot() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder", pivotEncoder.getPosition());
  }

  public Command Up(double speed) {
    return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }
  
}
