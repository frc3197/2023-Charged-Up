// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendAutomatic extends CommandBase {
  /** Creates a new ExtendAutomatic. */
  ArmSubsystem subsystem;
  PIDController extendPID;
  String distance;
  int GoalTicks;
  public ExtendAutomatic(ArmSubsystem subsystem, String distance) {
    this.subsystem = subsystem;
    this.distance = distance;
    extendPID = Constants.Arm.ExtendPID;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(distance.equals("far"))
    {
      GoalTicks = Constants.Arm.TICKS_TO_FAR;
    }
    else if(distance.equals("close"))
    {
      GoalTicks = Constants.Arm.TICKS_TO_CLOSE;
    }
    else
    {
      GoalTicks = 0;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subsystem.setExtending(true);
    //this.subsystem.setManuelMove(true);
    subsystem.extend(extendPID.calculate(subsystem.getTicks(), GoalTicks));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(subsystem.getTicks() - GoalTicks) < Constants.Arm.TICK_THRESHOLD) {
      subsystem.extend(0);
      this.subsystem.setExtending(false);
      //this.subsystem.setManuelMove(false);
      return true;
    }
    return false;
  }
}
