package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class PneumaticCommand extends CommandBase {
  private PneumaticSubsystem subsystem;

  public PneumaticCommand(PneumaticSubsystem system) {
    subsystem = system;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    subsystem.toggle();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}