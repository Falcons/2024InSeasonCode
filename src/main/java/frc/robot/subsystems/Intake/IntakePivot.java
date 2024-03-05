// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.pivotID, MotorType.kBrushless);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkAbsoluteEncoder TBEncoder = pivot.getAbsoluteEncoder();
  private final SparkLimitSwitch limit = pivot.getReverseLimitSwitch(Type.kNormallyOpen);
  public IntakePivot() {}

  public void set(double speed){
    if(limit.isPressed() && speed>0) {pivot.stopMotor(); return;}
    pivot.set(speed); 
  }

  public void stop(){
    pivot.stopMotor();
  }

  public double getPivotPosition(){
    return encoder.getPosition();
  }

  public double getThroughBorePosition(){
    return TBEncoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake pivot encoder", encoder.getPosition());
    SmartDashboard.putNumber("intake through bore encoder", TBEncoder.getPosition());
    SmartDashboard.putBoolean("intake pivot limit", limit.isPressed());
  }

  public Command Extend(double speed) {return this.startEnd(() -> this.set(-speed), () -> this.stop());}
  public Command Retract(double speed) {return this.startEnd(() -> this.set(speed), () -> this.stop());}
}
