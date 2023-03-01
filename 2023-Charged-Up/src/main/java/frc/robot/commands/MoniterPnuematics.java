// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class MoniterPnuematics extends CommandBase {
  /** Creates a new MoniterPnuematics. */
  PneumaticSubsystem subsystem;
  Timer timer;
  public MoniterPnuematics(PneumaticSubsystem subsystem) {
    this.subsystem = subsystem;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
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
    //System.out.println(subsystem.getCompressor());
    if(!subsystem.getCompressor())
    {
      timer.stop();
    }
    else
    {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Compressor ran for: " + timer.get());
    System.out.println("Num triggers: " + subsystem.getTriggers());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
