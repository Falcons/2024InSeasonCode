// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ApriltagID;
  NetworkTableEntry pipelineEntry;
  NetworkTableEntry tbotpose;
  double x;
  double y;
  double area;
  double getID;
  boolean tv;
  double []botPose = new double[6];

  public double getX(){
    return x;
  }

  public double getY(){
    return y;
  }

  public double getArea(){
    return area;
  }

  public double getID(){
    return getID;
  }

  public double []getBotPose() {
    return botPose;
  }

  public boolean getTV() {
    return tv;
  }

  public void setID(double id) {
    pipelineEntry.setNumber(id);
  }

  public Limelight(String key) {
    table = NetworkTableInstance.getDefault().getTable(key);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ApriltagID = table.getEntry("tid");
    pipelineEntry = table.getEntry("pipeline");
    tbotpose = table.getEntry("botpose");
  }

  @Override
  public void periodic() {
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    getID = ApriltagID.getDouble(0.0);
    botPose = tbotpose.getDoubleArray(new double[6]);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("ID", getID);
    SmartDashboard.putNumberArray("LimelightBotpose", botPose);
    SmartDashboard.getNumber("pipeline", (double) pipelineEntry.getNumber(0));
    }

  
}
