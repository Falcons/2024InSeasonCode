// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterPivot;

public class SetShooterPosition extends Command {
  private final ShooterPivot shooterpivot;
  private final double pos;
  private final PIDController pid;
  private final ArmFeedforward armFF;
  
  public SetShooterPosition(ShooterPivot shooterpivot, double pos) {
    this.shooterpivot = shooterpivot;
    this.pos = pos;
    this.pid = new PIDController(0, 0, 0);
    this.armFF = new ArmFeedforward(0, 0, 0, 0);
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
    double speed = (pid.calculate(shooterpivot.getPosition(), pos) +
                    armFF.calculate(shooterpivot.getPosition(), 0));
    shooterpivot.setVoltage(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
