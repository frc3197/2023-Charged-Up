// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignRRT extends CommandBase {
  /** Creates a new AlignRRT. */

  DrivetrainSubsystem driveSubsystem;
  Limelight limelightSubsystem;
  int desiredTag;
  double[] targetSpace;

  Rotation2d robotAngle;
  Rotation2d targetAngle;
  Pose2d robotPosition;
  Pose2d targetPose;

  double offset = -0.5;
  double thresh = 0.25;
  double degreesThresh = 1;
  double maxRotationSpeed = 0.4;
  double rotSpeed = 0;

  double maxAlignSpeed = 0.80;

  double visionMeasurement;
  

  public AlignRRT(Limelight limelightSubsystem, DrivetrainSubsystem subsystem) {
    this.driveSubsystem = subsystem;
    this.limelightSubsystem = limelightSubsystem;
    limelightSubsystem.setPipeline("tape");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    limelightSubsystem.setPipeline("tape");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    visionMeasurement = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    double xSpeed = visionMeasurement/-7.0;
    double ySpeed = driveSubsystem.getPoseEstimation().getX() * 3;

    if(xSpeed > maxAlignSpeed)
    {
      xSpeed = maxAlignSpeed;
    }
    if(xSpeed<maxAlignSpeed*-1)
    {
      xSpeed = maxAlignSpeed*-1;
    }
    if(ySpeed > maxAlignSpeed)
    {
      ySpeed = maxAlignSpeed;
    }
    if(ySpeed < maxAlignSpeed*-1)
    {
      ySpeed = maxAlignSpeed*-1;
    }
    
    if(Math.abs(visionMeasurement) < thresh)
    {
      xSpeed = 0;
    }

    driveSubsystem.drive(new ChassisSpeeds(0, xSpeed, rotSpeed));

    //0-2 are offset in meters, goes x, y, z, rest are rotations I assume hi mason
    
    // MOVE TO SUBSYSTEM, more organized

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

      return Math.abs(visionMeasurement) < thresh*1.25
      && Math.abs(driveSubsystem.getPoseEstimation().getY()) < 0.02;
   
  }
}
