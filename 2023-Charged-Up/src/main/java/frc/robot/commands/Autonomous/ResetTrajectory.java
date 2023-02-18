// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetTrajectory extends InstantCommand {
  DrivetrainSubsystem driveSubsystem;
  PathContainer pathContainer;
  PathPlannerTrajectory trajectory;
  PoseAtTime poseAtTime;

  public enum PoseAtTime{
    START,END
  }



  public ResetTrajectory(DrivetrainSubsystem driveSubsystem, PathContainer pathContainer, PoseAtTime poseAtTime) {
    this.driveSubsystem = driveSubsystem;
    this.pathContainer = pathContainer;
    this.poseAtTime = poseAtTime;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = PathPlanner.loadPath(pathContainer.getPathString(), pathContainer.getMaxSpeed(), pathContainer.getMaxAcceleration());
    if(poseAtTime.equals(PoseAtTime.START)){
      if(pathContainer.getFirst()){
        driveSubsystem.setOffset(-trajectory.getInitialState().holonomicRotation.getDegrees());
      }
    // DriveSubsystem.getGyroscopeObj().setAngleAdjustment(trajectory.getInitialState().holonomicRotation.getDegrees());
    driveSubsystem.resetOdometry(new Pose2d(trajectory.getInitialState().poseMeters.getTranslation(),trajectory.getInitialState().holonomicRotation));
    driveSubsystem.zeroGyroscope();
  }
  else{
    // DriveSubsystem.getGyroscopeObj().setAngleAdjustment(trajectory.getEndState().holonomicRotation.getDegrees());
    driveSubsystem.resetOdometry(new Pose2d(trajectory.getEndState().poseMeters.getTranslation(),trajectory.getEndState().holonomicRotation));

  }
}
}