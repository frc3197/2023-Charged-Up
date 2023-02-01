package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class PneumaticCommand extends CommandBase {
  private PneumaticSubsystem subsystem;
  private boolean once = false;

  public PneumaticCommand(PneumaticSubsystem system) {
    subsystem = system;
  }

  @Override
  public void initialize() {
    once = false;
  }

  @Override
  public void execute() {
    subsystem.toggleSingle();
    once = true;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return once;
  }
}
