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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivot extends SubsystemBase {
  private final CANSparkMax pivot = new CANSparkMax(ShooterConstants.pivotID,  MotorType.kBrushless);
  private final SparkAbsoluteEncoder thrubore = pivot.getAbsoluteEncoder();
  private final DigitalInput pivotbottomlimit = new DigitalInput(0);

  private final RelativeEncoder pivotEncoder = pivot.getEncoder();
  public ShooterPivot() {}

  public void setSpeed(double speed) {
    pivot.set(speed);
    
  }

  public void stopShooterPivot() {
    pivot.stopMotor();
  }

  public boolean getPivotLimit() {
    return pivotbottomlimit.get();
  }

  public double getThruBore() {
    return thrubore.getPosition();
  }

  public boolean getSoftLimit() {
    return (thrubore.getPosition() < 0.945);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder", thrubore.getPosition());
    SmartDashboard.putBoolean("Pivot Limit", getPivotLimit());
    SmartDashboard.putBoolean("Soft Limit Enabled", getSoftLimit());
  }

  public Command Up(double speed) {
      return this.startEnd(() -> this.setSpeed(-speed), () -> this.stopShooterPivot());
  }

  public Command Down(double speed) {
    return this.startEnd(() -> this.setSpeed(speed), () -> this.stopShooterPivot());
  }
  
}
