// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  WPI_TalonFX swivelMotor;
  WPI_TalonFX extendMotor;

  Encoder throughBore;

  public ArmSubsystem() {

    swivelMotor = new WPI_TalonFX(Constants.Arm.SWIVEL_MOTOR_ID);
    extendMotor = new WPI_TalonFX(Constants.Arm.EXTENTION_MOTOR_ID);

    throughBore = new Encoder(Constants.Arm.ENCODER_INPUT_ID, Constants.Arm.ENCODER_OUTPUT_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void swivel(double val)
  {
    swivelMotor.set(val);
  }

  public void extend(double val)
  {
    extendMotor.set(val);
  }

  public void resetEncoder()
  {
    throughBore.reset();
  }

  public int getTicks()
  {
    return throughBore.get();
  }
}
