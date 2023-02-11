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

public class AlignAT extends CommandBase {
  Limelight limelightSubsystem;
  DrivetrainSubsystem driveSubsystem;
  int desiredTag;
  double[] targetspace;

  Pose2d robotPosition;
  Pose2d targetPose;
  
  // Need some offset, don't bash into driver station tingz D:
  double tagOffset = 0.25;
  double thresh = 0.1;

  public AlignAT(Limelight light, DrivetrainSubsystem subsystem) {
    limelightSubsystem = light;
    // The arguement for pipelines should be either "april" or "tape"
    limelightSubsystem.setPipeline("april");
    driveSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   robotPosition = limelightSubsystem.getBotPose();
    targetspace = limelightSubsystem.getTargetSpace();
    targetPose = new Pose2d(targetspace[0], targetspace[2] + tagOffset, new Rotation2d(0.0, 0.0));
/*/
    double xSpeed = (robotPosition.getX() - targetPose.getX()) /6.0;    ;
    double ySpeed = (robotPosition.getY() - targetPose.getY()) /6.0;*/

    double xSpeed=0;
    double ySpeed=0;

    xSpeed = targetPose.getX() / 1.5;
    ySpeed = targetPose.getY() / -1.5;

    if(Math.abs(targetPose.getX()) < thresh) {
      xSpeed = 0;
    }

    if(Math.abs(targetPose.getY()) < thresh) {
      ySpeed = 0;
    }


    driveSubsystem.drive(new ChassisSpeeds(ySpeed, xSpeed, 0));

    System.out.println(xSpeed + ", " + ySpeed);
    //System.out.println("X: " + xSpeed * -6 + " & " + ("Y: " + ySpeed * 6));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetPose.getX() - robotPosition.getX()) < thresh && Math.abs(targetPose.getY() - robotPosition.getY()) < thresh;
  }
}