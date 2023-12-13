// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Vision;

public class SnapCube extends CommandBase {
  DrivetrainSubsystem driveSub;
  PoseEstimatorSubsystem poseSub;
  Vision visionSub;

  double maxSpeed = 1.5;
  double maxRot = 4.0;
  double thresh = 0.025;
  int index = 0;
  int num;
//1.145
  double[] redCube = new double[]{14.700, 4.414};
  double[] blueCube = new double[]{1.459, 4.414};
  double[][] locations = new double[][]{redCube, blueCube};

  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;

  public SnapCube(DrivetrainSubsystem drive, PoseEstimatorSubsystem pose, Vision vision, int chosenIndex) {
    driveSub = drive;
    poseSub = pose;
    visionSub = vision;
    this.index = chosenIndex;

    visionSub.setCancel(false);
    num = 0;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSub.setCancel(false);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    num ++;
    double yaw = 0;
    yaw = driveSub.getYaw() / -6;

    if(yaw > maxRot) {
      yaw = maxRot;
    }
    if(yaw < maxRot * -1) {
      yaw = maxRot * -1;
    }

    double rotSpeed = (Units.degreesToRadians(visionSub.getYaw()) - Units.degreesToRadians(0))/-10;
    if(rotSpeed > maxRot) {
      rotSpeed = maxRot;
    }
    if(rotSpeed < maxRot * -1) {
      rotSpeed = maxRot * -1;
    }

    rotSpeed = 0;
    

    double xSpeed = -4*(poseSub.getCurrentPose().getX() - locations[index][0]);
    System.out.println(poseSub.getCurrentPose().getX() - locations[index][0]);
    double ySpeed = -2*(poseSub.getCurrentPose().getY() - locations[index][1]);

    if(xSpeed > maxSpeed) {
      xSpeed = maxSpeed;
    }
    if(xSpeed < maxSpeed * -1) {
      xSpeed = maxSpeed * -1;
    }

    if(index >= 2) {
      xSpeed *= -1;
      ySpeed *= -1;
    }

    if(ySpeed > maxSpeed) {
      ySpeed = maxSpeed;
    }
    if(ySpeed < maxSpeed * -1) {
      ySpeed = maxSpeed * -1;
    }

    final double xVal = xSpeed;
    final double yVal = ySpeed;
    final double rotVal = yaw;


    m_translationXSupplier = () -> yVal;
    m_translationYSupplier = () -> xVal;
    m_rotationSupplier = () -> rotVal;
    
    
    driveSub.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationYSupplier.getAsDouble(),
            m_translationXSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble(),
            driveSub.getGyroscopeRotation()));

    //driveSub.drive(new ChassisSpeeds(xSpeed, ySpeed, yaw));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(visionSub.getCancel()) {
      driveSub.drive(new ChassisSpeeds(0, 0, 0));
      return true;
    }
    if(Math.abs(poseSub.getCurrentPose().getX() - locations[index][0]) < thresh && Math.abs(poseSub.getCurrentPose().getY() - locations[index][1]) < thresh) {
      driveSub.drive(new ChassisSpeeds(0, 0, 0));
      return true;
    }
    return false;
  }
}
