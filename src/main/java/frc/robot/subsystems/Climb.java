// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimb = new CANSparkMax(ClimbConstants.leftClimbID, MotorType.kBrushless);
  private final CANSparkMax rightClimb = new CANSparkMax(ClimbConstants.rightClimbID, MotorType.kBrushless);

  private final RelativeEncoder leftLiftEncoder = leftClimb.getEncoder();
  private final RelativeEncoder rightLiftEncoder = rightClimb.getEncoder();

  //private final DigitalInput leftClimbLimit = new DigitalInput(ClimbConstants.leftClimbLimitPort);
  //private final DigitalInput rightClimbLimit = new DigitalInput(ClimbConstants.rightClimbLimitPort);

  public Climb() {}

  public void setClimb(double leftSpeed, double rightSpeed) {
    leftClimb.set(leftSpeed);
    rightClimb.set(rightSpeed);
  }

  public void setLeftClimb(double speed){
    leftClimb.set(speed);
  }
  public void setRightClimb(double speed){
    rightClimb.set(speed);
  }

  public void stopClimb() {
    leftClimb.stopMotor();
    rightClimb.stopMotor();
  }
  
  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Left Climb Limit", leftClimbLimit.get());
    //SmartDashboard.putBoolean("Right Climb Limit", rightClimbLimit.get());
  }

  public Command Up(double speed) {
    return this.startEnd(() -> this.setClimb(speed, speed), () -> this.stopClimb());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setClimb(-speed, -speed), () -> this.stopClimb());
  }
}