// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class AlignGamepiece extends CommandBase {
  ArmSubsystem armSubsystem;
  DrivetrainSubsystem driveSubsystem;
  Vision vision;
  double pitch;
  double yaw;
  double maxPitch = 0.25;
  double maxYaw = 0.25;

  double xSpeed;
  double ySpeed;
  double yawSpeed;

  int counter = 0;

  public AlignGamepiece(ArmSubsystem aSub, DrivetrainSubsystem dSub, Vision vSub) {
    armSubsystem = aSub;
    driveSubsystem = dSub;
    vision = vSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(driveSubsystem.getYaw());
    if(vision.hasTarget() && (vision.getPitch()) < 5.5) {
      //counter = 0;
    } else {
      counter ++;
    }

    if((vision.getPitch()) < 4.5) {
      counter ++;
    } else {
      counter = 0;
    }

    pitch = vision.getPitch();
    yaw = vision.getYaw();
    //System.out.println(pitch);
    double pitchSpeed = pitch /20;
    if(pitchSpeed > maxPitch) {
      pitchSpeed = maxPitch;
    }
    if(pitchSpeed < maxPitch * -1) {
      pitchSpeed = maxPitch * -1;
    }

    yawSpeed = yaw / -6;
    //armSubsystem.swivel(pitchSpeed);
    double botYaw = driveSubsystem.getYaw();
    xSpeed = Math.cos(Units.degreesToRadians(botYaw)) * pitchSpeed;
    ySpeed = Math.sin(Units.degreesToRadians(botYaw)) * pitchSpeed;
    if(botYaw > 90 && botYaw < 270) {
      xSpeed *= -1;
    }
    System.out.println(xSpeed + ", " + ySpeed);
    driveSubsystem.drive(new ChassisSpeeds(xSpeed*4, ySpeed*4, yawSpeed));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.swivel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(xSpeed) < 0.2 && Math.abs(ySpeed) < 0.2 && Math.abs(yawSpeed) <0.2 && counter > 20;
  }
}
