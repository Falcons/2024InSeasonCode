// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;


public class MoveToShooter extends Command {
    private final Drivetrain drivetrain;
    private final PIDController pid;
    private final PIDController gyroPID;
    private final Limelight limelight;
    private double []bp;
    private ADIS16470_IMU gyro = new ADIS16470_IMU();
  
    public MoveToShooter(Drivetrain drivetrain, double []bp, Limelight limelight) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.pid = new PIDController(0.05, 0, 0);
        this.gyroPID = new PIDController(0.05, 0, 0);
        this.limelight = limelight;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!limelight.getTV()) {
      drivetrain.arcadeDrive(0, 0.1);
    }
    bp = this.limelight.getBotPose();

    double robotAngle = bp[5];
    double xToMove = bp[0] - 0.87;
    double yToMove = bp[1] - (-5.6);
    double hypMove = Math.sqrt(Math.pow(yToMove, 2) + Math.pow(xToMove, 2));
    double angleMove = Math.acos(yToMove/hypMove) + robotAngle;

    if (hypMove > 0) {
       drivetrain.arcadeDrive(-pid.calculate(hypMove, 0), gyroPID.calculate(angleMove, 0));
    }
    else {
       drivetrain.arcadeDrive(0,0);
       SmartDashboard.putNumber("distance", hypMove);
    }
    drivetrain.arcadeDrive(-pid.calculate(0, 0), gyroPID.calculate(gyro.getAngle(), 0));
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