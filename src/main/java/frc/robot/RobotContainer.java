// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveForward;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ManualIntakePivot;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Climb climb = new Climb();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new IntakeNote(intake, -0.3));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new IntakeNote(intake, 1));
    new JoystickButton(driver, XboxController.Button.kX.value).whileTrue(new Shoot(shooter, 0.5, 0.5));
    new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(new Shoot(shooter, 1, 0.95));
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new ManualIntakePivot(intake, 0.5));
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new ManualIntakePivot(intake, -0.8));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
