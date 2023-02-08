package frc.robot.commands.Arm;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

public class Swivel extends CommandBase {
  /** Creates a new Swivel. */
  ArmSubsystem m_subsystem;
  double val;
  double maxVal;
  int GoalTicks;
  PIDController swivelPID;
  Encoder throughBore;
  ArmFeedforward feedforward;
  CommandXboxController controller;
  int targetAxis = -1;
   
  public Swivel(ArmSubsystem m_subsystem, double val, CommandXboxController controller, int axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = m_subsystem;
    this.maxVal = val;
    this.controller = controller;
    swivelPID = Constants.Arm.LevelPID;
    //feedforward = new ArmFeedforward(0, 0, 0);
    this.targetAxis = axis;

    addRequirements(m_subsystem);
  }

  public Swivel(ArmSubsystem m_subsystem, double val) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = m_subsystem;
    this.maxVal = val;
    swivelPID = Constants.Arm.LevelPID;
    //feedforward = new ArmFeedforward(0, 0, 0);

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.targetAxis != -1) {
      this.val = this.maxVal * controller.getRawAxis(this.targetAxis);
    } else {
      this.val = this.maxVal;
    }
      m_subsystem.swivel(val);
      System.out.println(m_subsystem.getTicks());
        //swivelPID.calculate(m_subsystem.getTicks(), GoalTicks);

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