// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  WPI_TalonFX intakeExtendMotor;
  WPI_TalonFX intakeSpinMotor;

  public IntakeSubsystem() {
    intakeExtendMotor = new WPI_TalonFX(Constants.Intake.MOTOR_DEPLOY_ID);
    intakeSpinMotor = new WPI_TalonFX(Constants.Intake.MOTOR_SPIN_ID);
  }

  public void deployIntake(double speed) {
    intakeExtendMotor.setVoltage(speed);
  }

  public void spinIntake(double val)
  {
    intakeSpinMotor.set(val);
  }
  
  // MILES will it use limit switch? probably  Whaddup whaddup?

@Override
  public void periodic() {
    // This method will be called once per scheduler run 
  }
}
