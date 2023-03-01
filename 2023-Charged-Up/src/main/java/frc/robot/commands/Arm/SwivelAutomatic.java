// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class SwivelAutomatic extends CommandBase {
  /** Creates a new SwivelAutomatic. */

  ArmSubsystem subsystem;
  PneumaticSubsystem pneumaticSubsystem;
  String level;
  double GoalTicks;
  int extendGoal;
  PIDController levelPID;
  ArmFeedforward feedforward;
  //Timer timer;
  double timeout;
  double delay = 0;
  boolean reachedHeight = false;
  boolean swivelFirst;

  public SwivelAutomatic(ArmSubsystem subsystem, PneumaticSubsystem pneumaticSubsystem, String level,
      boolean swivelFirst) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    this.swivelFirst = swivelFirst;
    this.subsystem = subsystem;
    this.pneumaticSubsystem = pneumaticSubsystem;
    levelPID = Constants.Arm.LevelPID;
    timeout = 3.5;

    //timer = new Timer();
    feedforward = new ArmFeedforward(
        Constants.Arm.KS,
        Constants.Arm.KG,
        Constants.Arm.KV,
        Constants.Arm.KA);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (level.equals("high")) {
      GoalTicks = Constants.Arm.TICKS_TO_HIGH;
      extendGoal = Constants.Arm.TICKS_TO_FAR_EXTEND;
    } else if (level.equals("mid")) {
      GoalTicks = Constants.Arm.TICKS_TO_MID;
      extendGoal = Constants.Arm.TICKS_TO_CLOSE_EXTEND;
    } else if (level.equals("low")) {
      GoalTicks = Constants.Arm.TICKS_TO_BOTTOM;
      extendGoal = 100;
    } else if (level.equals("substation")) {
      GoalTicks = Constants.Arm.TICKS_TO_SUBSTATION;
      extendGoal = 100;
    } else {
      GoalTicks = 6.4;
      extendGoal = 10;
    }

    //timer.stop();
    //timer.reset();
    //timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (swivelFirst) {
      
    } else {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.swivel(0);
    subsystem.extend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (/*timer.get() > timeout*/false) {
      return true;
    }

    if (Math.abs(subsystem.mapAbsoluteEncoder() - GoalTicks) < Constants.Arm.TICK_THRESHOLD
        && Math.abs(subsystem.getExtendTicks() - extendGoal) < Constants.Arm.EXTEND_TICK_THRESHOLD) {
      subsystem.swivel(0);
      this.subsystem.setMove(false);
      this.subsystem.setManuelMove(false);
      return true;
    }
    return false;
  }
}
