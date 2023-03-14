// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class Extend extends CommandBase {
  /** Creates a new Extend. */
  ArmSubsystem m_subsystem;
  double val;
  ElevatorFeedforward feedforward;
  

  public Extend(ArmSubsystem m_subsystem, double val) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = m_subsystem;
    this.val = val;
    feedforward = new ElevatorFeedforward(Constants.Arm.EXTEND_KS, Constants.Arm.EXTEND_KG, Constants.Arm.EXTEND_KV, Constants.Arm.EXTEND_KS);
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedVal = feedforward.calculate(0) / 4.0;

    if(m_subsystem.getExtendTicks() < 100000 || val < 0)
    {
    m_subsystem.extend(val);
    m_subsystem.setExtending(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setExtending(false);
    m_subsystem.extend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
