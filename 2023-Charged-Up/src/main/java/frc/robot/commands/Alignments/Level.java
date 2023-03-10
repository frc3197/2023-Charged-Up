// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Level extends CommandBase {
  /** Creates a new Level. */
  int direction = 1;
  double levelSpeed = 1;
  double thresh = 5;

  DrivetrainSubsystem subsystem;
  public Level(DrivetrainSubsystem sub, int dir) {
    subsystem = sub;
    direction = dir;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(subsystem.getRoll());
    double speed = 0;
    if(subsystem.getRoll() > thresh) {
      speed = levelSpeed;
    }
    if(subsystem.getRoll() < thresh * -1) {
      speed = levelSpeed * -1;
    }
    speed = subsystem.getRoll() / 15;
    speed *= -0.95;
    System.out.println("SPEED: " + speed);
    subsystem.drive(new ChassisSpeeds(speed, 0.0, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
