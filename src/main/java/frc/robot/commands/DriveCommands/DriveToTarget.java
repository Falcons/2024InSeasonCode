// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.commands.Limelight;
import frc.robot.subsystems.Limelight;

public class DriveToTarget extends Command {
  /** Creates a new driveToTarget. */
  private final Limelight limelight;
  private final Drivetrain drivetrain;
  private final PIDController pid;
  private final PIDController gyroPID;
  private double note_x;
  private double note_y;
  private double note_distanceHyp;
  private String type;
  private String object;
  private double targetOffsetAngleVertical;
  private double ID;

  
  public DriveToTarget(Drivetrain drivetrain, String type, String object, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.type = type;
    this.object = object;
    this.pid = new PIDController(0.05, 0, 0);
    this.gyroPID = new PIDController(0.05, 0, 0);
    this.limelight = limelight;
    addRequirements(drivetrain, limelight);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (object.equals("tag")) {
      limelight.setID(1);

    } else if (object.equals("note")) {
      limelight.setID(0);
    }

    if (!limelight.getTV()) {
      drivetrain.arcadeDrive(0, 0.1);
    }

    this.note_x = limelight.getX();
    if (type.equals("limelight-intake")) {
      this.note_y = limelight.getY();
    } else {
      this.note_y = limelight.getY()+30;
    }
    

    // vertical angle of target to center
    targetOffsetAngleVertical = this.note_y;

    // height of limelight lens from ground in inches
    double limelightLensHeightInches = type.equals("limelight-intake") ? 8 : 8; //change
    // height of target from ground in inches
    double targetHeight;
    if (type.equals("limelight-intake")) {
       targetHeight = 0;
    } else {
        double []heights = {51.375, 51.375, 55.125, 55.125, 51.375, 51.375, 55.125, 55.125, 51.375, 51.375, 50.75, 50.75, 50.75, 50.75, 50.75, 50.75};
        targetHeight = heights[(int) ID];
    }

    // horizontal angle towards target 
    double angleToGoalRadians = targetOffsetAngleVertical * (Math.PI / 180);
    // outward distance towards target
    double note_distance = (targetHeight - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    // distance towards target
    note_distanceHyp = note_distance/Math.cos(note_x * (Math.PI / 180));

    
    if (note_distanceHyp > 20) {
      // center and move towards note
      drivetrain.arcadeDrive(-pid.calculate(note_distanceHyp, 20), gyroPID.calculate(note_x, 0));
    } else {
      // stop moving if distance is less than 20 inches
      drivetrain.arcadeDrive(0,0);
      SmartDashboard.putNumber("distance", note_distanceHyp);
    }
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