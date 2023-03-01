// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendAutomatic extends CommandBase {
  /** Creates a new ExtendAutomatic. */
  ArmSubsystem subsystem;
  PIDController extendPID;
  int distance;
  double GoalTicks;
  double delay;
  int num = 0;
  Timer timer;
  double timeout = 0.5;

  public ExtendAutomatic(ArmSubsystem subsystem, int distance, double delay) {
    this.subsystem = subsystem;
    this.distance = distance;
    extendPID = Constants.Arm.ExtendPID;
    GoalTicks = distance;
    this.delay = delay;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    {
      this.subsystem.setExtending(true);
      // this.subsystem.setManuelMove(true);
      subsystem.extend(Constants.Arm.ExtendPID.calculate(subsystem.getExtendTicks(), GoalTicks) / 200.0);
      System.out.println(Constants.Arm.ExtendPID.calculate(subsystem.getExtendTicks(), GoalTicks) / 200.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > timeout)
    {
      return true;
    }
    if (Math.abs(subsystem.getExtendTicks() - GoalTicks) < Constants.Arm.EXTEND_TICK_THRESHOLD) {
      subsystem.extend(0);
      this.subsystem.setExtending(false);
      // this.subsystem.setManuelMove(false);
      return true;
    }
    return false;
  }
}
