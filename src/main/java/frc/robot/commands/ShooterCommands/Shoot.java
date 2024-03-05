// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Flywheel;

public class Shoot extends Command {
  private final Flywheel flywheel;
  private double leftspeed;
  private double rightspeed;
  public Shoot(Flywheel flywheel, double leftspeed, double rightspeed) {
    this.flywheel = flywheel;
    this.leftspeed = leftspeed;
    this.rightspeed = rightspeed;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.set(leftspeed, rightspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.set(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
