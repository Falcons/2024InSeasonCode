// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Wheels;

public class IntakeNote extends Command {
  private final Wheels wheels;
  private final double speed;
  public IntakeNote(Wheels wheels, double speed) {
    this.wheels = wheels;
    this.speed = speed;
    addRequirements(wheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wheels.set(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wheels.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wheels.hasNote();
  }
}
