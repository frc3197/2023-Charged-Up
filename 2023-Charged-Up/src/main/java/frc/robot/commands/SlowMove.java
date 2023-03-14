// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SlowMove extends CommandBase {
  /** Creates a new SlowMove. */
  DrivetrainSubsystem subsystem;
  ChassisSpeeds chassisSpeeds;
  public SlowMove(DrivetrainSubsystem subsystem, ChassisSpeeds chassisSpeeds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.chassisSpeeds = chassisSpeeds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {subsystem.drive(chassisSpeeds);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {subsystem.drive(new ChassisSpeeds());}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
