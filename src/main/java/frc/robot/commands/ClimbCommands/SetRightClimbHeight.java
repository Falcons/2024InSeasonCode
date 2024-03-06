// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.RightClimb;

public class SetRightClimbHeight extends Command {
  /** Creates a new SetRightClimbHight. */
  public final RightClimb climb;
  public final PIDController pid = new PIDController(0.05, 0, 0);
  public final double height;
  public SetRightClimbHeight(RightClimb climb, double height) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.height = height;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.set(pid.calculate(climb.getPosition(), height));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
