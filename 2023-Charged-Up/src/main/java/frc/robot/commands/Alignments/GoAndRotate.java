// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GoAndRotate extends CommandBase {
  /** Creates a new GoAndRotate. */
  double degrees;
  double maxRot = 3;
  double rotSpeed = 0;
  double yLength;
  double xLength;
  double maxSpeed = 1.65;
  double xVal;
  double rollThresh = 7.5;
  DrivetrainSubsystem subsystem;

  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;

  public GoAndRotate(double yLength, double xLength, double degrees, DrivetrainSubsystem subsystem, double xVal, double rollThresh, double speed) {
    this.degrees = degrees;
    this.subsystem = subsystem;
    this.yLength = yLength;
    this.xVal = xVal;
    this.rollThresh = rollThresh;
    this.maxSpeed = speed;
    this.xLength = xLength;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotSpeed = ((subsystem.getYaw()) - degrees) / -10;
    if (rotSpeed > maxRot) {
      rotSpeed = maxRot;
    }
    if (rotSpeed < maxRot * -1) {
      rotSpeed = maxRot * -1;
    }

    double currentSpeed = yLength / 3.0;
    if (currentSpeed > maxSpeed) {
      currentSpeed = maxSpeed;
    }
    if (currentSpeed < maxSpeed * -1) {
      currentSpeed = maxSpeed * -1;
    }
    // System.out.println("CURRENT SPEED:" + currentSpeed + ", LENGTH: " + length);
    final double test = currentSpeed;
    double xSpeed =  xLength;
    if(xSpeed > maxSpeed * 1.2) {
      xSpeed = maxSpeed * 1.2;
    }
    if(xSpeed < maxSpeed * -1.2) {
      xSpeed = maxSpeed * -1.2;
    }
    final double xSpeedFinal = xSpeed;
    m_translationXSupplier = () -> xSpeedFinal;
    m_translationYSupplier = () -> test;
    m_rotationSupplier = () -> rotSpeed;
    
    xLength -= xSpeed;
    
    subsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationYSupplier.getAsDouble(),
            m_translationXSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble(),
            subsystem.getGyroscopeRotation()));
            yLength -= currentSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(subsystem.getRoll()) > Math.abs(rollThresh)) {
      return true;
    }
    return Math.abs(rotSpeed) < 0.5 && Math.abs(yLength) < 0.1 && Math.abs(xLength) < 0.1;
  }
}
