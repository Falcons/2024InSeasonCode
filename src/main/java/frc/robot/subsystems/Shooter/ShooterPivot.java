// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
  private final SparkAbsoluteEncoder thruBore = pivot.getAbsoluteEncoder();
  private final DigitalInput pivotBottomLimit = new DigitalInput(0);

  public ShooterPivot() {
    thruBore.setPositionConversionFactor(ShooterConstants.pivotThruBoreToRadians);
  }
  

  public void setSpeed(double speed) {
    pivot.set(speed);
  }

  public void setVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  public void stopShooterPivot() {
    pivot.stopMotor();
  }

  public boolean getPivotLimit() {
    return pivotBottomLimit.get();
  }

  public double getThruBore() {
    return thruBore.getPosition();
  }

  public boolean getSoftLimit() {
    return (thruBore.getPosition() > 0.945);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder", thruBore.getPosition());
    SmartDashboard.putBoolean("Pivot Limit Switch", getPivotLimit());
    SmartDashboard.putBoolean("Soft Limit Enabled", getSoftLimit());
  }

  public Command Up(double speed) {
      return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  } 
}
