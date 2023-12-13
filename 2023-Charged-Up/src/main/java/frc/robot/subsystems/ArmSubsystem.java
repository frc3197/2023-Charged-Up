// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  // WPI_TalonFX swivelMotor;
  // WPI_TalonFX extendMotor;

  CANSparkMax swivelMotor;
  CANSparkMax extendMotor;

  RelativeEncoder swivelEncoder;
  RelativeEncoder extEncoder;

  double maxSwivelSpeed = Constants.Arm.SWIVEL_SPEED;
  double maxExtendSpeed = Constants.Arm.EXTEND_SPEED;

  private boolean isMoving = false;
  private boolean cancelManuel = false;

  private boolean extending = false;

  int divideAmount = 150000;

  boolean autoWrist = true;

  // Encoder throughBore = new Encoder();
  DutyCycleEncoder encoder;

  // RelativeEncoder extendEncoder;

  public ArmSubsystem() {
    swivelMotor = new CANSparkMax(Constants.Arm.SWIVEL_MOTOR_ID, MotorType.kBrushless);
    extendMotor = new CANSparkMax(Constants.Arm.EXTENTION_MOTOR_ID, MotorType.kBrushless);

    extEncoder = extendMotor.getEncoder();

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
    if (val > maxExtendSpeed) {
      val = maxExtendSpeed;
    }
    if (val < -maxExtendSpeed) {
      val = -maxExtendSpeed;
    }
    extendMotor.set(val);
    //System.out.println("ENCODER EXTEND: " + extEncoder.getPosition());
  }

  public void resetEncoder() {
    // encoder.reset();
    // for(int i = 0; i < 100; i ++)
    // System.out.println("Reset Encoder: " + encoder.getAbsolutePosition());
  }

  public double getSpeed() {
    // System.out.print("RATE: " + throughBore.getRate());
    return 0;
  }

  public double getTicks() {
    encoder.setPositionOffset(Constants.Arm.SWIVEL_ABSOLUTE_OFFSET);
    // System.out.println(throughBore.get() * -1);
    // System.out.println(encoder.isConnected());
    return encoder.getAbsolutePosition() - encoder.getPositionOffset();
  }

  public double getRad() {
    return ((double) encoder.getAbsolutePosition() * 1 / 2048.0) * Math.PI * 2.0;
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

  public double getExtendTicks() {
    return extEncoder.getPosition();
  }

  public double mapAbsoluteEncoder() {
    double startValue = encoder.getAbsolutePosition() * 10;
    if (startValue > 5) {
      startValue -= 5;
    } else {
      startValue += 5;
    }
    return startValue;
  }

  public void zeroExtendEncoder() {
    extEncoder.setPosition(0);
  }

  public void setDivide(int num) {
    divideAmount = num;
  }

  public int getDivide() {
    return divideAmount;
  }

  public void setMaxSpeed(double val) {
    maxSwivelSpeed = Constants.Arm.SWIVEL_SPEED * val;
    maxExtendSpeed = Constants.Arm.EXTEND_SPEED * val;
  }

  public void setMaxExtend(double val)
  {
    maxExtendSpeed = val;
  }

  public void toggleAutoWrist() {
    autoWrist = !autoWrist;
    SmartDashboard.putBoolean("Auto-Wrist", autoWrist);
  }

  public boolean getAutoWrist() {
    return autoWrist;
  }

}
