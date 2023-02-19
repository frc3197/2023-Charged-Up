// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  WPI_TalonFX swivelMotor;
  WPI_TalonFX extendMotor;

  double maxSwivelSpeed = 0.15;

  private boolean isMoving = false;
  private boolean cancelManuel = false;

  private boolean extending = false;

  //Encoder throughBore = new Encoder();
  DutyCycleEncoder encoder;
  
  //RelativeEncoder extendEncoder;

  public ArmSubsystem() {
    swivelMotor = new WPI_TalonFX(Constants.Arm.SWIVEL_MOTOR_ID);
    extendMotor = new WPI_TalonFX(Constants.Arm.EXTENTION_MOTOR_ID);

    extendMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    extendMotor.setSelectedSensorPosition(0);

    encoder = new DutyCycleEncoder(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void swivel(double val) {
    if (val > maxSwivelSpeed) {
      val = maxSwivelSpeed;
    }
    if (val < maxSwivelSpeed * -1) {
      val = maxSwivelSpeed * -1;
    }
    swivelMotor.set(val);
  }

  public void extend(double val) {
    extendMotor.set(val);
    System.out.println("ENCODER EXTEND: " + extendMotor.getSelectedSensorPosition());
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public double getSpeed() {
   // System.out.print("RATE: " + throughBore.getRate());
    return 0;
  }

  public double getTicks() {
    //System.out.println(throughBore.get() * -1);
    System.out.println(encoder.isConnected());
    return encoder.getAbsolutePosition();
  }

  public double getRad() {
    return ((double)encoder.getAbsolutePosition() * 1/2048.0) * Math.PI * 2.0;
  }

  public void setMove(boolean bool) {
    isMoving = bool;
  }

  public boolean getMove() {
    return isMoving;
  }

  public void setManuelMove(boolean bool) {
    cancelManuel = bool;
  }

  public boolean getManuelMove() {
    return cancelManuel;
  }

  public void setExtending(boolean bool) {
    extending = bool;
  }

  public boolean getExtending() {
    return extending;
  }

  public double getExtendTicks()
  {
    return extendMotor.getSelectedSensorPosition();
  }
}
