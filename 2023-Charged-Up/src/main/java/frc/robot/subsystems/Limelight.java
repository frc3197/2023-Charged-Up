// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  double[] pose;
  NetworkTableEntry pipelineEntry;
  boolean on = false;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable(Constants.Limelight.limelightName);
    pipelineEntry = table.getEntry("pipeline");
  }

  public double[] getTargetSpace() {
    pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    System.out.println(pose);
    return pose;
  }

  public void setPipeline(String pipeline) {
    switch(pipeline) {
      case "april":
        pipelineEntry.setNumber(0);
        break;
      case "tape":
        pipelineEntry.setNumber(1);
        break;
      default: break;
    }
  }

  public double getRot() {
    return table.getEntry("botpose").getDoubleArray(new double[6])[5];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getTargets() {
    System.out.println(table.getEntry("tid").getDouble(-1));
    if(table.getEntry("tid").getDouble(-1) >= 0) {
      return true;
    }
    return false;
  }

  public boolean getLimeTargets()
  {
  if(table.getEntry("limelight").getDouble(-1) >= 0)
  {
    return true;
  }
  return false;
  }

  public void toggleLimelight()
  {
    on = !on;

    if(on)
    {
    table.getEntry("ledMode").setNumber(1);
    }
    else{
      table.getEntry("ledMode").setNumber(3);
    }
  }
}