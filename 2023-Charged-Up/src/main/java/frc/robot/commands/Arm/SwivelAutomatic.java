// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class SwivelAutomatic extends CommandBase {
  /** Creates a new SwivelAutomatic. */

  ArmSubsystem subsystem;
  PneumaticSubsystem pneumaticSubsystem;
  String level;
  double GoalTicks;
  PIDController levelPID;
  ArmFeedforward feedforward;

  public SwivelAutomatic(ArmSubsystem subsystem, PneumaticSubsystem pneumaticSubsystem, String level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    this.subsystem = subsystem;
    this.pneumaticSubsystem = pneumaticSubsystem;
    levelPID = Constants.Arm.LevelPID;
    feedforward = new ArmFeedforward(
      Constants.Arm.KS, 
      Constants.Arm.KG,
      Constants.Arm.KV,
      Constants.Arm.KA);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(level.equals("high"))
    {
      GoalTicks = Constants.Arm.TICKS_TO_HIGH;
    }
    else if(level.equals("mid"))
    {
      GoalTicks = Constants.Arm.TICKS_TO_MID;
    }
    else if (level.equals("low"))
    {
      GoalTicks = Constants.Arm.TICKS_TO_BOTTOM;
    }
    else 
    {
      GoalTicks = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(subsystem.mapAbsoluteEncoder() - GoalTicks);
    this.subsystem.setMove(true);
    this.subsystem.setManuelMove(true);
    subsystem.swivel(levelPID.calculate(subsystem.mapAbsoluteEncoder(), GoalTicks) * -1);

    if(subsystem.mapAbsoluteEncoder() > 5 && subsystem.mapAbsoluteEncoder() < 6.5) {
      pneumaticSubsystem.closeClaw();
      //once = false;
    } else {
      pneumaticSubsystem.enteringZone();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(subsystem.mapAbsoluteEncoder() - GoalTicks) < Constants.Arm.TICK_THRESHOLD) {
      subsystem.swivel(0);
      this.subsystem.setMove(false);
      this.subsystem.setManuelMove(false);
      return true;
    }
    return false;
  }
}
