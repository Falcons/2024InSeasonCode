// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

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

  HashMap<Double, Double> shooterPivotMap = new HashMap<Double, Double>();

  public ShooterPivot() {
    //thruBore.setZeroOffset(ShooterConstants.thruBoreZeroOffset);
    thruBore.setPositionConversionFactor(1);
  }
  
  public void setShooterPivotMap() {
    shooterPivotMap.put(0.0, 0.954);
    shooterPivotMap.put(0.1524, 0.947);
    shooterPivotMap.put(0.3048, 0.933);
    shooterPivotMap.put(0.4572, 0.935);
    shooterPivotMap.put(0.6069, 0.926);
    shooterPivotMap.put(0.7620, 0.922);
    shooterPivotMap.put(0.9144, 0.915);
    shooterPivotMap.put(1.0668, 0.9111);
    shooterPivotMap.put(1.219, 0.909);
    shooterPivotMap.put(1.3716, 0.906);
    shooterPivotMap.put(1.524, 0.898);
    shooterPivotMap.put(1.6764, 0.898);
    shooterPivotMap.put(1.8288, 0.898);
    shooterPivotMap.put(1.9304, 0.898);
  }

  public double getHash(double key) {
    return shooterPivotMap.get(key);
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

  public boolean getSoftUpperLimit() {
    return (thruBore.getPosition() > 0.945);
  }

  public boolean getSoftLowerLimit() {
    return (thruBore.getPosition() < 0.88);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder", thruBore.getPosition());
    SmartDashboard.putBoolean("Pivot Limit Switch", getPivotLimit());
    SmartDashboard.putBoolean("Soft Limit Enabled", getSoftUpperLimit());
  }

  public Command Up(double speed) {
      return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  } 
}
