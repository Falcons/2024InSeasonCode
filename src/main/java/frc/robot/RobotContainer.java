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

  //Trigger pivotLimitSwitch = new Trigger(shooterpivot::getPivotLimit);

  public RobotContainer() {
    drivetrain.setDefaultCommand(
      new RunCommand(() -> drivetrain.arcadeDrive(
        -driver.getLeftY(), 
        driver.getRightX()),
         drivetrain));

    leftClimb.setDefaultCommand(new RunCommand(() -> leftClimb.set(-operator.getLeftY())));
    rightClimb.setDefaultCommand(new RunCommand(() -> rightClimb.set(-operator.getLeftY())));
    configureBindings();
  }

  private void configureBindings() {
    // Intake and Eject
    operator.b().whileTrue(intakeWheels.EjectNoteCmd(1));
    operator.x().onTrue(new IntakeNote(intakeWheels, 1)); //temp/testing
    operator.y().onTrue(new ExtendThenIntake(intakePivot, intakeWheels)); //temp/testing

    operator.leftBumper().whileTrue(intakePivot.Retract(0.3));
    operator.rightBumper().whileTrue(intakePivot.Extend(0.3));

    //shooter
    operator.rightTrigger().whileTrue(shooterFlywheel.Shoot(operator.getRightTriggerAxis(), operator.getRightTriggerAxis()*0.95));

    // Shooter Pivot

    //driver.b().whileTrue(shooterpivot.Up(0.1));
    driver.b().whileTrue(new Up(shooterpivot, 0.1)); // moves shooter up //0.0125 value to hold pos from encoder value 0.85 to < 0.945 0.945 - 0.955 hold with vaule 0 0.955 < - 0.99 hold  pos with opposite 0.0125, move shoooter with 0.025 value (up) for down let gravity take it
    
    if (!shooterpivot.getLimit()) {
      driver.x().whileTrue(shooterpivot.Down(0.1));
    }
    //driver.x().and(pivotLimitSwitch.negate()).onTrue(shooterpivot.Down(0.1));
    //driver.x().whileTrue(new Down(shooterpivot, 0.1));

    // Climb

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
