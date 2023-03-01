// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetLevelMode extends CommandBase {
  /** Creates a new SetLevelMode. */
  ArmSubsystem subsystem;
  String piece;
  public SetLevelMode(ArmSubsystem subsystem, String piece) {
    this.subsystem = subsystem;
    this.piece = piece;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch(piece)
    {
      case "cube":
      subsystem.setDivide(239000);
      break;
      case "cone":
      subsystem.setDivide(195000);
      break;
      default:
      subsystem.setDivide(240000);
      break;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
