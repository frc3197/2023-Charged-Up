// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignCharge extends CommandBase {
  Limelight limelightSubsyetem;
  DrivetrainSubsystem driveSubsystem;
  double ySpeed;
  double distance;
  double maxSpeed = 0.75;
  double thresh = 0.15;

  public AlignCharge(Limelight sub, DrivetrainSubsystem train) {
    limelightSubsyetem = sub;
    driveSubsystem = train;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = limelightSubsyetem.getTargetSpace()[2];
    System.out.println(distance);
    if(!limelightSubsyetem.getTargets()) {
      //ySpeed = -0.5;
    }
    ySpeed = (distance + 1.6) * -1;
    ySpeed = -1;
    if(ySpeed > maxSpeed) {
      ySpeed = maxSpeed;
    }
    if(ySpeed < maxSpeed * -1) {
      ySpeed = maxSpeed * -1;
    }
    driveSubsystem.drive(new ChassisSpeeds(ySpeed, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(distance + 1.75) < thresh;
  }
}
