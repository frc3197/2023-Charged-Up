// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class NewSwivel extends CommandBase {
  /** Creates a new NewSwivel. */
  ArmSubsystem subsystem;
  PneumaticSubsystem subp;
  int count = 0;
  double val;

  public NewSwivel(ArmSubsystem sub, PneumaticSubsystem subP, double num) {
    subsystem = sub;
    val = num;
    subp = subP;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
    if (count > 120) {
      //subsystem.swivel(-0.1);
    } else if(count < 240){
      //subsystem.swivel(0.1);
    } else { 
      //subsystem.swivel(0);
    }
    if(count == 415) {
      //count = 0;
      subp.toggleClaw();
      
    }

    if(count > 620) {
      subp.toggleWrist();
      count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
