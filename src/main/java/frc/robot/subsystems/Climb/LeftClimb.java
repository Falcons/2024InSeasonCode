// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class LeftClimb extends SubsystemBase {
  /** Creates a new LeftClimb. */
  private final CANSparkMax climb = new CANSparkMax(ClimbConstants.leftClimbID, MotorType.kBrushless);
  private final DigitalInput limit = new DigitalInput(ClimbConstants.leftLimitSwitchID);
  private final RelativeEncoder encoder = climb.getEncoder();
  public LeftClimb() {}
  
  public void set(double speed){
    if(limit.get() && speed<0) {climb.stopMotor(); return;}
    climb.set(speed);
  }

  public void stop(){
    climb.stopMotor();
  }

  public double getPosition(){
    return encoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left climb encoder", encoder.getPosition());
    SmartDashboard.putBoolean("left climb limit", limit.get());
  }

  public Command Up(double speed) {return this.startEnd(() -> this.set(speed), () -> this.stop());}
  public Command Down(double speed) {return this.startEnd(() -> this.set(-speed), () -> this.stop());}
}
