// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveForward;
import frc.robot.commands.IntakeCommands.ExtendThenIntake;
import frc.robot.commands.IntakeCommands.IntakeNote;
import frc.robot.commands.IntakeCommands.ManualIntakePivot;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.ShooterCommands.Down;
import frc.robot.commands.ShooterCommands.SetShooterPosition;
import frc.robot.commands.ShooterCommands.Up;
import frc.robot.subsystems.Climb.LeftClimb;
import frc.robot.subsystems.Climb.RightClimb;
import frc.robot.subsystems.Intake.Wheels;
import frc.robot.subsystems.Shooter.ShooterPivot;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter.Flywheel;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final LeftClimb leftClimb = new LeftClimb();
  private final RightClimb rightClimb = new RightClimb();
  private final Wheels intakeWheels = new Wheels();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Flywheel shooterFlywheel = new Flywheel();
  private final ShooterPivot shooterpivot = new ShooterPivot();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    drivetrain.setDefaultCommand(
      new ArcadeDriveCmd(drivetrain, 
      () -> -driver.getLeftY(), 
      () -> -driver.getRightX()));

    leftClimb.setDefaultCommand(new RunCommand(() -> leftClimb.set(-operator.getLeftY())));
    rightClimb.setDefaultCommand(new RunCommand(() -> rightClimb.set(-operator.getLeftY())));
    configureBindings();
  }

  private void configureBindings() {
    shooterpivot.stop();

    driver.leftBumper().whileTrue(
      new RunCommand(() -> drivetrain.arcadeDriveManual(
        -driver.getLeftY(), 
        -driver.getRightX()),
         drivetrain));

  
    // Intake wheels manual
    // Intake and Eject
    operator.b().whileTrue(intakeWheels.EjectNoteCmd(1));
    // Intake Manual
    //operator.povUp().whileTrue(intake.Extend(0.3));
    //operator.povDown().whileTrue(intake.Retract(0.3));

    //Intake PID Test
    operator.povUp().onTrue(new Extend(intakePivot));
    operator.povDown().onTrue(new Retract(intakePivot, intakeWheels));

    //intake compound
    operator.rightBumper().onTrue(
      new Extend(intakePivot)
      .andThen(intakeWheels.IntakeNoteCmd(0.3)));

    // Shooter
    operator.x().whileTrue(shooterFlywheel.Shoot(0.5, 0.5));
    operator.y().whileTrue(shooterFlywheel.Shoot(1, 0.95));


    // shooter variable
    operator.leftTrigger(0.5).whileTrue(shooterFlywheel.Shoot(0.5, 0.5).unless(operator.leftTrigger(0.9)));
    operator.leftTrigger(0.9).whileTrue(shooterFlywheel.Shoot(1, 0.95));

    // Shooter Pivot Manual
    driver.b().whileTrue(new Up(shooterpivot, 0.1)); // moves shooter up //0.0125 value to hold pos from encoder value 0.85 to < 0.945 0.945 - 0.955 hold with vaule 0 0.955 < - 0.99 hold  pos with opposite 0.0125, move shoooter with 0.025 value (up) for down let gravity take it
    driver.x().whileTrue(new Down(shooterpivot, 0.1));
    //driver.x().and(pivotLimitSwitch.negate()).onTrue(shooterpivot.Down(0.1));

    //Shooter PID test
    operator.leftBumper().whileTrue(new SetShooterPosition(shooterpivot, 0.95).unless(shooterpivot::getSoftLimit));
    // Climb

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
