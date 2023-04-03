package frc.robot.commands.Arm;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

public class Swivel extends CommandBase {
  /** Creates a new Swivel. */
  ArmSubsystem m_subsystem;
  double val;
  double maxVal;
  int GoalTicks;
  PIDController swivelPID;
  // Encoder throughBore;
  ArmFeedforward feedforward;
  CommandXboxController controller;
  PneumaticSubsystem pneumatics;
  int targetAxis = -1;
  boolean once = false;

  public Swivel(ArmSubsystem m_subsystem, PneumaticSubsystem p, double val, CommandXboxController controller,
      int axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = m_subsystem;
    this.maxVal = val;
    this.controller = controller;
    swivelPID = Constants.Arm.LevelPID;
    // feedforward = new ArmFeedforward(0, 0, 0);
    this.targetAxis = axis;
    pneumatics = p;

    addRequirements(m_subsystem);
  }

  public Swivel(ArmSubsystem m_subsystem, double val) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = m_subsystem;
    this.maxVal = val;
    swivelPID = Constants.Arm.LevelPID;
    // feedforward = new ArmFeedforward(0, 0, 0);

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setMove(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.targetAxis != -1) {
      this.val = this.maxVal * controller.getRawAxis(this.targetAxis);
    } else {
      this.val = this.maxVal;
    }

    if (m_subsystem.mapAbsoluteEncoder() > 5 && m_subsystem.mapAbsoluteEncoder() < 6.5) {
      // pneumatics.closeClaw();
      once = false;
    } else {
      // pneumatics.enteringZone();
    }
    /*
     * if(m_subsystem.mapAbsoluteEncoder() < 5 && m_subsystem.mapAbsoluteEncoder() >
     * 6.5 && !once) {
     * once = true;
     * if(!pneumatics.getPrevious()) {
     * pneumatics.openClaw();
     * }
     * }
     */

    if (m_subsystem.getAutoWrist()) {
      if (m_subsystem.mapAbsoluteEncoder() < Constants.Arm.TICKS_TO_WRIST) {
        pneumatics.closeWrist();
      } else {
        pneumatics.openWrist();
      }
    }

    m_subsystem.swivel(val);
    System.out.println("ENCODER-SWIVEL: " + m_subsystem.mapAbsoluteEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.swivel(0);
    m_subsystem.setMove(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Finished if the tick value is within the threshold
    return false;
    // Math.abs(m_subsystem.getTicks() - GoalTicks) < Constants.Arm.TICK_THRESHOLD;
  }
}