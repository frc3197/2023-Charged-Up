// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  NetworkTable table;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry pipelineEntry;

  String pipeline;

  public Limelight(String pipelineType) {
    table = NetworkTableInstance.getDefault().getTable("limelight-mason");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    pipeline = pipelineType;
    pipelineEntry = table.getEntry("pipeline");
    setPipeline();
  }

  public String getOutput() {
    return (tx.getDouble(0.0) + ", " + ty.getDouble(0.0) + ", " + ta.getDouble(0.0));
  }

  private void setPipeline() {
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
}