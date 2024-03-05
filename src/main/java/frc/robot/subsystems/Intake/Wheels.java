// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Wheels extends SubsystemBase {
  /** Creates a new Wheels. */
  private final CANSparkMax wheels = new CANSparkMax(IntakeConstants.wheelID, MotorType.kBrushless);
  private final RelativeEncoder encoder = wheels.getEncoder();
  public Wheels() {}

  public void set(double speed){
    wheels.set(speed);
  }

  public void stop(){
    wheels.stopMotor();
  }
  
  public double getPosition(){
    return encoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake wheels endcoder", encoder.getPosition());
  }

  public Command IntakeNoteCmd(double speed) {return this.startEnd(() -> this.set(-speed), () -> this.stop());}
  public Command EjectNoteCmd(double speed) {return this.startEnd(() -> this.set(speed), () -> this.stop());}
}
