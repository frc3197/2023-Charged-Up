package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Vision;

public class ClawPneumatic extends CommandBase {
  private PneumaticSubsystem subsystem;
  Vision visionSub;

  public ClawPneumatic(PneumaticSubsystem system, Vision vision) {
    subsystem = system;
    visionSub = vision;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    subsystem.toggleClaw();
    visionSub.setCancel(true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}