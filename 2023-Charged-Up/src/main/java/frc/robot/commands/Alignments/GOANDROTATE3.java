// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class GOANDROTATE3 extends CommandBase {
  /** Creates a new GoAndRotate. */
  double degrees;
  double maxRot = 3;
  double rotSpeed = 0;
  double length;
  //1.7
  double maxSpeed = 1.4;
  double xVal;
  double rollThresh = 20;
  DrivetrainSubsystem subsystem;

  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;

  public GOANDROTATE3(double length, double degrees, DrivetrainSubsystem subsystem, double xVal, double rollThresh) {
    this.degrees = degrees;
    this.subsystem = subsystem;
    this.length = length;
    this.xVal = xVal;
    this.rollThresh = rollThresh;
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
    rotSpeed = 0;

    double currentSpeed = length / 3.0;
    if (currentSpeed > maxSpeed) {
      currentSpeed = maxSpeed;
    }
    if (currentSpeed < maxSpeed * -1) {
      currentSpeed = maxSpeed * -1;
    }
    double ySpeed = Math.cos(Units.degreesToRadians(subsystem.getYaw())) * currentSpeed;
    // System.out.println("CURRENT SPEED:" + currentSpeed + ", LENGTH: " + length);
    final double test = currentSpeed;
    m_translationXSupplier = () -> xVal;
    m_translationYSupplier = () -> test;
    m_rotationSupplier = () -> rotSpeed;
    
    
    subsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationYSupplier.getAsDouble(),
            m_translationXSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble(),
            subsystem.getGyroscopeRotation()));
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
    if(Math.abs(subsystem.getRoll()) > Math.abs(rollThresh)) {
      return true;
    }
    return Math.abs(rotSpeed) < 0.5 && Math.abs(length) < 0.1;
  }
}
