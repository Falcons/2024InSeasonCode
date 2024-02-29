// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private final CANSparkMax leftClimb = new CANSparkMax(ClimbConstants.leftClimbID, MotorType.kBrushless);
  private final CANSparkMax rightClimb = new CANSparkMax(ClimbConstants.rightClimbID, MotorType.kBrushless);

  private final RelativeEncoder leftLiftEncoder = leftClimb.getEncoder();
  private final RelativeEncoder rightLiftEncoder = rightClimb.getEncoder();

  public Climb() {}

  public void setClimb(double speed) {
    leftClimb.set(speed);
    rightClimb.set(speed);
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
    // This method will be called once per scheduler run
  }

  public Command Up(double speed) {
    return this.startEnd(() -> this.setClimb(speed), () -> this.stopClimb());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setClimb(-speed), () -> this.stopClimb());
  }
}
