// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  //NetworkTable table;
  //NetworkTableInstance instance;
  PhotonCamera cameraFront;

  boolean cancelDrive = false;

  /** Creates a new Vision. */
  public Vision() {

    //table = NetworkTableInstance.getDefault().getTable("photonvision");
    //instance = table.getInstance();
    cameraFront = new PhotonCamera("front_camera");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPitch() {
    var results = cameraFront.getLatestResult();
    if (results.hasTargets()) {
      var target = results.getBestTarget();
      return target.getPitch();
    }
    return 0.0;
  }

  public void setPipeline(int num) {
    cameraFront.setPipelineIndex(num);
    for (int i = 0; i < 100; i++)
      System.out.println("PIPELINE: " + cameraFront.getPipelineIndex());
  }

  public boolean hasTarget() {
    var results = cameraFront.getLatestResult();
    return results.hasTargets();
  }

  

  public double getYaw() {
    var results = cameraFront.getLatestResult();
    if (results.hasTargets()) {
      var target = results.getBestTarget();
      return target.getYaw();
    }
    return 0.0;
  }

  public double getArea() {
    var results = cameraFront.getLatestResult();
    if (results.hasTargets()) {
      var target = results.getBestTarget();
      System.out.print(target.getArea()
      );
      return target.getArea();
    }
    return 0.0;
  }

  public void setCancel(boolean value) {
    cancelDrive = value;
  }

  public boolean getCancel() {
    return cancelDrive;
  }
}
