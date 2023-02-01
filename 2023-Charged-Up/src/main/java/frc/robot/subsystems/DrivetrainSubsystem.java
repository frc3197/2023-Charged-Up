// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {

    CANCoder frontLeftSteerEncoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_ENCODER_ID);
    CANCoder frontRightSteerEncoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_ENCODER_ID);
    CANCoder backLeftSteerEncoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_ENCODER_ID);
    CANCoder backRightSteerEncoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_ENCODER_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
