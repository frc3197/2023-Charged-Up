// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinIntake extends CommandBase {
  /** Creates a new RunIntake. */
  IntakeSubsystem subsystem;
  double val;

  public SpinIntake(IntakeSubsystem subsystem, double val) {
    this.subsystem = subsystem;
    this.val = val;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    subsystem.spinIntake(val);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
