// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Go extends CommandBase {
  /** Creates a new Go. */
  DrivetrainSubsystem subsystem;
  double length;
  double maxSpeed = 1.5;
  double maxRot = 0.65;

  public Go(DrivetrainSubsystem subsystem, double length) {
    this.subsystem = subsystem;
    this.length = length;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("GO: " + subsystem.getYaw());
    double rotSpeed = (subsystem.getYaw()%180) /-15;
    double currentSpeed = length / 3.0;
    if(currentSpeed > maxSpeed) {
      currentSpeed = maxSpeed;
    }
    if(currentSpeed < maxSpeed * -1) {
      currentSpeed = maxSpeed * -1;
    }
    if(rotSpeed > maxRot) {
      rotSpeed = maxRot;
    }
    if(rotSpeed < maxRot * -1) {
      rotSpeed = maxRot * -1;
    }
    subsystem.drive(new ChassisSpeeds(currentSpeed,0,rotSpeed));
    length -= currentSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(length) < 0.1;
  }
}
