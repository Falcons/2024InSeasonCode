// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class SetShooterPosition extends Command {
  private final ShooterPivot shooterpivot;
  private final PIDController pid;

  public SetShooterPosition(ShooterPivot shooterpivot) {
    this.shooterpivot = shooterpivot;
    this.pid = new PIDController(0, 0, 0);
    addRequirements(shooterpivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pid.calculate(0);
    //shooterpivot.setSpeed()
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
