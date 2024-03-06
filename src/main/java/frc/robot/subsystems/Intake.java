// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(IntakeConstants.pivotID, MotorType.kBrushless);
  private final CANSparkMax wheels = new CANSparkMax(IntakeConstants.wheelID, MotorType.kBrushless);

  private final SparkAbsoluteEncoder intakeThruBore = pivot.getAbsoluteEncoder();

  private final DigitalInput intakeLimit = new DigitalInput(IntakeConstants.intakeBottomLimit);

  public Intake() {}

  public void IntakeNote(double speed){
    wheels.set(-speed);
  }

  public void EjectNote(double speed) {
    wheels.set(speed);
  }

  public void stopIntake() {
    wheels.stopMotor();
  }

  public void pivotSpeed(double speed) {
    pivot.set(speed);
  }

  public void stopIntakePivot() {
    pivot.stopMotor();
  }

  public double getIntakeAngle() {
    return intakeThruBore.getPosition();
  }

  public boolean getBottomSoftLimit() {
    return (intakeThruBore.getPosition() > IntakeConstants.intakeInAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Thru Bore", intakeThruBore.getPosition());
    SmartDashboard.putBoolean("Intake Limit", intakeLimit.get());
  }

  public Command IntakeNoteCmd(double speed) {
    return this.startEnd(() -> this.IntakeNote(speed), () -> this.stopIntake());
  }

  public Command EjectNoteCmd(double speed) {
    return this.startEnd(() -> this.EjectNote(speed), () -> this.stopIntake());
  }

  public Command Extend(double speed) {
    return this.startEnd(() -> this.pivotSpeed(-speed), () -> this.stopIntakePivot());
  }

  public Command Retract(double speed) {
    return this.startEnd(() -> this.pivotSpeed(speed), () -> this.stopIntakePivot());
  }
}