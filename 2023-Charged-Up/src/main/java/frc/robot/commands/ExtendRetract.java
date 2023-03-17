// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendRetract extends CommandBase {
  /** Creates a new ExtendRetract. */
  ArmSubsystem subsystem;
  
  double val;
  int num;
  
  public ExtendRetract(ArmSubsystem subsystem, double val) {
    this.subsystem = subsystem;
    
    this.val = val;
    num = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    num = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.extend(val);
    System.out.println(num);
    num++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    subsystem.extend(0);
    num = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return num>25;
  }
}
