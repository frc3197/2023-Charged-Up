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

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable(Constants.Limelight.limelightName);
    pipelineEntry = table.getEntry("pipeline");
  }

  public double[] getTargetSpace() {
    pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    return pose;
  }

  public void setPipeline(String pipeline) {
    switch(pipeline) {
      case "april":
        pipelineEntry.setNumber(Constants.Limelight.APRIL_TAG_PIPELINE_INDEX);
        break;
      case "tape":
        pipelineEntry.setNumber(Constants.Limelight.RETRO_REFLECTIVE_TAPE_PIPELINE_INDEX);
        break;
      default: break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Pose2d getBotPose()
  {
    double[] temp = table.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose2d(temp[0], temp[2], new Rotation2d(temp[5]));
  }

  public Rotation2d getBotRotation()
  {
    double[] temp = table.getEntry("botpose").getDoubleArray(new double[6]);
    return new Rotation2d(temp[5]);
  }

  public boolean getTargets() {
    System.out.println(table.getEntry("tid").getDouble(-1));
    if(table.getEntry("tid").getDouble(-1) >= 0) {
      return true;
    }
    return false;
  }
}