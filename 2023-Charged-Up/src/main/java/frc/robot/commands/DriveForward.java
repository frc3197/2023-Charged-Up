// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class DriveForward extends CommandBase {
  /** Creates a new DriveForward. */
  DrivetrainSubsystem subsystem;
  double speed;
  Vision visionSub;

  public DriveForward(DrivetrainSubsystem sub, double speed, Vision vis) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = sub;
    this.speed = speed;
    visionSub = vis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.drive(new ChassisSpeeds(speed, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
