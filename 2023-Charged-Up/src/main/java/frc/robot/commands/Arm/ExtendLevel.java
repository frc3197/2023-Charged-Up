// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendLevel extends CommandBase {

  ElevatorFeedforward feedforward;
  ArmSubsystem subsystem;
  
  public ExtendLevel(ArmSubsystem sub) {
    feedforward = new ElevatorFeedforward(Constants.Arm.EXTEND_KS, Constants.Arm.EXTEND_KG, Constants.Arm.EXTEND_KV, Constants.Arm.EXTEND_KS);
    subsystem = sub;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!subsystem.getExtending()) {
      double feedVal = feedforward.calculate(0) / 4.0;
      subsystem.extend(feedVal);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.getExtending();
  }
}
