// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
  private final DigitalInput limit = new DigitalInput(0);
  // private final SparkLimitSwitch limit = pivot.getForwardLimitSwitch(Type.kNormallyOpen);
  private final RelativeEncoder pivotEncoder = pivot.getEncoder();
  public ShooterPivot() {}

  public void set(double speed) {
    if(limit.get() && speed<0){pivot.stopMotor(); return;}
    pivot.set(speed);
  }

  public void stop() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder", pivotEncoder.getPosition());
  }

  public Command Up(double speed) {
    return this.startEnd(() -> this.set(-speed), () -> this.stop());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.set(speed), () -> this.stop());
  }
  
}
