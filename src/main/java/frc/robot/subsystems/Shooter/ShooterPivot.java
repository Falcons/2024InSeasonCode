// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
  private final DigitalInput limit = new DigitalInput(0);
  private final RelativeEncoder pivotEncoder = pivot.getEncoder();
  private final SparkAbsoluteEncoder thruBore = pivot.getAbsoluteEncoder();

  public ShooterPivot() {
    thruBore.setPositionConversionFactor(ShooterConstants.pivotThruBoreToRadians);
  }

  public void set(double speed) {
    // if(limit.get() && speed<0){pivot.stopMotor(); return;}
    pivot.set(speed);
  }
  public void setVoltage(double voltage){
    pivot.setVoltage(voltage);
  }

  public void stop() {
    pivot.stopMotor();
  }
  public boolean getLimit(){
    return limit.get();
  }
  public double getPosition(){
    return thruBore.getPosition();
  }
  public boolean getSoftLimit(){
    return (thruBore.getPosition() > 0.945);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder", pivotEncoder.getPosition());
    SmartDashboard.putNumber("thru bore", thruBore.getPosition());
    SmartDashboard.putBoolean("shooter pivot limit", limit.get());
  }

  public Command Up(double speed) {
    return this.startEnd(() -> this.set(-speed), () -> this.stop());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.set(speed), () -> this.stop());
  }
  
}
