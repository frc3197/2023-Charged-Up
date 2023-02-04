package frc.robot.commands.Arm;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Encoder;

public class Swivel extends CommandBase {
  /** Creates a new Swivel. */
  ArmSubsystem m_subsystem;
  String level;
  double val;
  int GoalTicks;
  PIDController swivelPID;
  Encoder throughBore;
   
  public Swivel(ArmSubsystem m_subsystem, String level, double val) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = m_subsystem;
    this.level = level;
    this.val = val;
    swivelPID = Constants.Arm.LevelPID;

    addRequirements(m_subsystem);
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

      m_subsystem.swivel(val);
        //swivelPID.calculate(m_subsystem.getTicks(), GoalTicks));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.swivel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //Finished if the tick value is within the threshold
    return false;
    //Math.abs(m_subsystem.getTicks() - GoalTicks) < Constants.Arm.TICK_THRESHOLD;
  }
}