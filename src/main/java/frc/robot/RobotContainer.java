// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterCommands.Down;
import frc.robot.commands.ShooterCommands.Up;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Climb climb = new Climb();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final ShooterPivot shooterpivot = new ShooterPivot();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  //Trigger pivotLimitSwitch = new Trigger(shooterpivot::getPivotLimit);

  public RobotContainer() {
    drivetrain.setDefaultCommand(
      new RunCommand(() -> drivetrain.arcadeDrive(
        -driver.getLeftY(), 
        driver.getRightX()),
         drivetrain));

    climb.setDefaultCommand(
      new RunCommand(() -> climb.setClimb(
      -operator.getLeftY(), 
      -operator.getRightY()),
       climb));

    configureBindings();
  }

  private void configureBindings() {
    // Intake and Eject
    operator.a().whileTrue(intake.IntakeNoteCmd(0.3));
    operator.b().whileTrue(intake.EjectNoteCmd(1));

    // Intake Pivot
    operator.leftBumper().whileTrue(intake.Extend(0.3));
    operator.rightBumper().whileTrue(intake.Retract(0.3));

    // Shooter
    operator.x().whileTrue(shooter.Shoot(0.5, 0.5));
    operator.y().whileTrue(shooter.Shoot(1, 0.95));

    // Shooter Pivot

    //driver.b().whileTrue(shooterpivot.Up(0.1));
    driver.b().whileTrue(new Up(shooterpivot, 0.1)); // moves shooter up //0.0125 value to hold pos from encoder value 0.85 to < 0.945 0.945 - 0.955 hold with vaule 0 0.955 < - 0.99 hold  pos with opposite 0.0125, move shoooter with 0.025 value (up) for down let gravity take it
    
    if (!shooterpivot.getPivotLimit()) {
      driver.x().whileTrue(shooterpivot.Down(0.1));
    }
    //driver.x().and(pivotLimitSwitch.negate()).onTrue(shooterpivot.Down(0.1));
    //driver.x().whileTrue(new Down(shooterpivot, 0.1));

    // Climb
    driver.y().whileTrue(climb.Up(0.2));
    driver.a().whileTrue(climb.Down(0.2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
