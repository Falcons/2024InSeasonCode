// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.Wheels;

public class Retract extends Command {
  private final IntakePivot intakePivot;
  private final Wheels intakeWheels;
  private final PIDController pid;
  public Retract(IntakePivot intakePivot , Wheels intakeWheels) {
    this.intakePivot = intakePivot;
    this.intakeWheels = intakeWheels;
    this.pid = new PIDController(1, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeWheels.stop();
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -pid.calculate(intakePivot.getIntakeAngle(), IntakeConstants.intakeInAngle);
    intakePivot.set(speed);
    SmartDashboard.putNumber("error", pid.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePivot.getBottomSoftLimit();
  }
}
