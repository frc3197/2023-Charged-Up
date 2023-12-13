// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Level extends CommandBase {
  /** Creates a new Level. */
  int rotGoal;
  double levelSpeed = 1;
  double thresh = 10.8;
  double maxRotSpeed = 0.125;
  // double currRoll;
  // double desiredRoll;

  DrivetrainSubsystem subsystem;

  public Level(DrivetrainSubsystem sub, int rotGoal) {
    subsystem = sub;
    this.rotGoal = rotGoal;
    // currRoll = subsystem.getRoll();
    // desiredRoll = currRoll - 1;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = 0;
    speed = subsystem.getRoll() / -29.8;
    if (subsystem.getRoll() < thresh && subsystem.getRoll() > thresh * -1) {
      speed = 0;
    }

    double rotSpeed = (rotGoal - subsystem.getYaw());

    if (rotSpeed > maxRotSpeed) {
      rotSpeed = maxRotSpeed;
    }
    if (rotSpeed < maxRotSpeed * -1) {
      rotSpeed = maxRotSpeed * -1;
    }
    //System.out.println("Roll: " + subsystem.getYaw());
    subsystem.drive(new ChassisSpeeds(speed, 0.0, 0));

    /*
     * 
     * 
     * 
     * 
     * if(desiredRoll > currRoll)
     * {
     * STOPPPP
     * subsystem.drive(new ChassisSpeeds(0, 0, 0));
     * }
     * else
     * {
     * keep moving forward very slowly
     * subsystem.drive(new ChassisSpeeds(0, getRoll() / 50.0, 0));
     * }
     * 
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
