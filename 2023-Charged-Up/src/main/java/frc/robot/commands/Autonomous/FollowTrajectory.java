// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */

  private ProfiledPIDController rotPID;
  private HolonomicDriveController hController;
  private PathPlannerState state;
  private Pose2d currentPose;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private PathContainer pathContainer;
  private PathPlannerTrajectory path;

  private final Timer timer = new Timer();

  DrivetrainSubsystem m_Subsystem;
  public FollowTrajectory(DrivetrainSubsystem m_Subsystem, PathContainer pathContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Subsystem = m_Subsystem;
    this.pathContainer = pathContainer;
    rotPID = Constants.Drivetrain.auto.ROT_PID_CONTROLLER;
    path = pathContainer.getTrajectory();
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
    hController = new HolonomicDriveController(Constants.Drivetrain.auto.X_PID_CONTROLLER, Constants.Drivetrain.auto.Y_PID_CONTROLLER, rotPID);

    timer.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var currTime = timer.get();
    state = (PathPlannerState) path.sample(currTime);
    currentPose = m_Subsystem.getPose();
    speeds = hController.calculate(currentPose, state, state.holonomicRotation);
    m_Subsystem.updateStates(m_Subsystem.getKinematics().toSwerveModuleStates(speeds));
  }

  public double getTime()
  {
    return timer.get();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Subsystem.updateStates(m_Subsystem.getKinematics().toSwerveModuleStates(new ChassisSpeeds()));
    timer.stop();
  }

  // Returns true when the command should end.
  
}
