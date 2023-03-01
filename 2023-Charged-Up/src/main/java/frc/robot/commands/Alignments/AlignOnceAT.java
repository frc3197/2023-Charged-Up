// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignOnceAT extends CommandBase {
  Limelight limelightSubsystem;
  DrivetrainSubsystem driveSubsystem;
  int desiredTag;
  double[] targetspace;

  Rotation2d robotAngle;
  Rotation2d targetAngle;
  Pose2d robotPosition;
  Pose2d targetPose;

  // Need some offset, don't bash into driver station tingz D:
  double tagOffsetX = 0.5;
  double tagOffsetY = 0.5;
  double thresh = 0.2;
  double degreesThresh = 1;
  double maxRotationSpeed = 0.4;
  double rotSpeed;

  double distanceX;
  double distanceY;

  double maxAlignSpeed = 0.85;
  boolean search;

  public AlignOnceAT(Limelight light, DrivetrainSubsystem subsystem, double offsetX, double offsetY) {
    limelightSubsystem = light;
    // The arguement for pipelines should be either "april" or "tape"
    limelightSubsystem.setPipeline("april");
    driveSubsystem = subsystem;
    tagOffsetX = offsetX;
    tagOffsetY = offsetY;

    targetspace = limelightSubsystem.getTargetSpace();
    targetPose = new Pose2d(targetspace[0]+ tagOffsetX, targetspace[2] + tagOffsetY, new Rotation2d(0.0, 0.0));

    targetAngle = new Rotation2d(targetspace[5]);

    distanceX = targetspace[0];
    distanceY = targetspace[2];
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // robotAngle = limelightSubsystem.getBotRotation();
    // robotPosition = limelightSubsystem.getBotPose();

    /*
     * /
     * double xSpeed = (robotPosition.getX() - targetPose.getX()) /6.0; ;
     * double ySpeed = (robotPosition.getY() - targetPose.getY()) /6.0;
     */

    double xSpeed = 0;
    double ySpeed = 0;

    // Rotation2d botRot = driveSubsystem.getGyroscopeRotation();
    // double rotation = botRot.getRadians();
    // System.out.println(botRot);
    // if(rotation > Math.PI / 2)

    xSpeed = distanceX * 1.5;
    ySpeed = distanceY * -1.5;
    rotSpeed = limelightSubsystem.getRot() * -1.5;

    if (search  && Math.abs(driveSubsystem.getRoll()) < 2) {
      ySpeed = -0.85;
    }

    if(Math.abs(driveSubsystem.getRoll()) > 1.5) {
      rotSpeed = 0;
      xSpeed = 0;
      ySpeed = -0.85;
    }

    if (Math.abs(targetPose.getX()) < thresh && Math.abs(driveSubsystem.getRoll()) < 2) {
      xSpeed = 0;
     }

    if (Math.abs(targetPose.getY()) < thresh && Math.abs(driveSubsystem.getRoll()) < 2) {
      ySpeed = 0;
    }

    if (Math.abs(targetAngle.getDegrees()) < degreesThresh) {
      // rotSpeed = 0;
    }

    // System.out.println(targetAngle + ", " + rotSpeed);

    if (xSpeed > maxAlignSpeed) {
      xSpeed = maxAlignSpeed;
    }
    if (xSpeed < maxAlignSpeed * -1) {
      xSpeed = maxAlignSpeed * -1;
    }

    if (ySpeed > maxAlignSpeed) {
      ySpeed = maxAlignSpeed;
    }
    if (ySpeed < maxAlignSpeed * -1) {
      ySpeed = maxAlignSpeed * -1;
    }

    if (rotSpeed > maxRotationSpeed) {
      rotSpeed = maxRotationSpeed;
    }

    if (rotSpeed < maxRotationSpeed * -1) {
      rotSpeed = maxRotationSpeed * -1;
    }

    System.out.println(targetspace[2]);
    driveSubsystem.drive(new ChassisSpeeds(ySpeed, xSpeed, 0));

    distanceX -= xSpeed;
    distanceY -= ySpeed;

    // System.out.println(xSpeed + ", " + ySpeed);
    // System.out.println("X: " + xSpeed * -6 + " & " + ("Y: " + ySpeed * 6));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!limelightSubsystem.getTargets() && !search) {
      //System.out.println("FINISHED 1");
      return true;
    }

    if (Math.abs(driveSubsystem.getRoll()) > 10) {
      System.out.println("FINISHED 2");
      return true;
      
    }

    if (limelightSubsystem.getTargets()) {
      return Math.abs(targetPose.getX()) < thresh * 1.25 && Math.abs(targetPose.getY()) < thresh
          && Math.abs(rotSpeed) < 3;
    }
    return false;
  }
}