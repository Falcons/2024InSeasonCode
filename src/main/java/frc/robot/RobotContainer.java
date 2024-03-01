// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveForward;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ManualIntakePivot;
import frc.robot.commands.Shoot;
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
    configureBindings();
  }

  private void configureBindings() {
    //intake
    operator.a().whileTrue(intakeWheels.IntakeNoteCmd(0.3));
    operator.b().whileTrue(intakeWheels.EjectNoteCmd(1));

    operator.leftBumper().whileTrue(intakePivot.Extend(0.3));
    operator.rightBumper().whileTrue(intakePivot.Retract(0.3));

    //shooter
    operator.x().whileTrue(shooterFlywheel.Shoot(0.5, 0.5));
    operator.y().whileTrue(shooterFlywheel.Shoot(1, 0.95));

    driver.b().whileTrue(shooterpivot.Up(0.2));
    driver.x().whileTrue(shooterpivot.Down(0.2));
    

    //climb
    driver.y().whileTrue(leftClimb.Up(0.2));
    driver.y().whileTrue(rightClimb.Up(0.2));
    driver.a().whileTrue(leftClimb.Down(0.2));
    driver.a().whileTrue(rightClimb.Down(0.2));

    /*
    //intake
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new IntakeNote(intake, -0.3));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new IntakeNote(intake, 1));

    //shooter
    new JoystickButton(driver, XboxController.Button.kX.value).whileTrue(new Shoot(shooter, 0.5, 0.5));
    new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(new Shoot(shooter, 1, 0.95));

    //intake pivot
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new ManualIntakePivot(intake, 0.5));
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new ManualIntakePivot(intake, -0.8));
    */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
