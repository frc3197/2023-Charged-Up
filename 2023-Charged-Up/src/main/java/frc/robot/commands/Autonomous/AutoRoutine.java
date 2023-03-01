// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine extends SequentialCommandGroup {
  /** Creates a new AutoRoutine. */

  private static DrivetrainSubsystem m_driveSubsystem = RobotContainer.getDriveSubsystem();
  private static ArmSubsystem m_armSubsystem = RobotContainer.getArmSubsystem();
  private static PneumaticSubsystem m_pneumaticSubsystem = RobotContainer.getPneumaticSubsystem();
  private static Limelight m_limelightSubsystem = RobotContainer.getLimelightSubsystem();


  public AutoRoutine(Command... commands) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(commands);
    addRequirements(m_driveSubsystem);
    addRequirements(m_armSubsystem);
    addRequirements(m_limelightSubsystem);
  }


public DrivetrainSubsystem getDriveSubsystem()
{
  return m_driveSubsystem;
}

public ArmSubsystem getArmSubsystem()
{
  return m_armSubsystem;
}

public PneumaticSubsystem getPneumaticSubsystem()
{
  return m_pneumaticSubsystem;
}

public Limelight getLimelightSubsystem()
{
  return m_limelightSubsystem;
}

}
