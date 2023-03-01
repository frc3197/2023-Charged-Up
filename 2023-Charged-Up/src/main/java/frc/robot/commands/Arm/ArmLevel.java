// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmLevel extends CommandBase {
  /** Creates a new ArmLevel. */

  ArmSubsystem subsystem;
  ArmFeedforward feed;
  double num;

  public ArmLevel(ArmSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    feed = new ArmFeedforward(
        Constants.Arm.KS,
        Constants.Arm.KG,
        Constants.Arm.KV,
        Constants.Arm.KA);

        addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!this.subsystem.getMove()) {
      //double val = feed.calculate(subsystem.getRad() - (Math.PI * (0.5)), 0.0, 0.0);
      //System.out.println("Feed Forward: " + val / (Math.PI * 2) / 2.0);
      subsystem.getSpeed();

      double val = subsystem.getExtendTicks() / (double) subsystem.getDivide();

      //Randians to Quaternians
      val *= Constants.Arm.LEVEL_CONSTANT;

      //subsystem.swivel((val / (Math.PI * 2)));
      //System.out.println("LEVELING: " + (val / (Math.PI * 2)));
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
