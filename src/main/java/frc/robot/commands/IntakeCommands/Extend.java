// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot;

public class Extend extends Command {
  /** Creates a new Extend. */
  private final IntakePivot intakePivot;
  public Extend(IntakePivot intakePivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePivot.set(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePivot.getLimit();
  }
}
