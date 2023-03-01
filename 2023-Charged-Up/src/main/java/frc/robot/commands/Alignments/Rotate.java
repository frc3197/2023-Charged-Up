// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Rotate extends CommandBase {
  double degrees;
  double maxRot = 3;
  double rotSpeed = 0;

  DrivetrainSubsystem subsystem;
  public Rotate(double degrees, DrivetrainSubsystem subsystem) {
    this.degrees = degrees;
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotSpeed = ((subsystem.getYaw()) - degrees) /-10;
    if(rotSpeed > maxRot) {
      rotSpeed = maxRot;
    }
    if(rotSpeed < maxRot * -1) {
      rotSpeed = maxRot * -1;
    }
    subsystem.drive(new ChassisSpeeds(0, 0, rotSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rotSpeed) < 0.5;
  }
}
