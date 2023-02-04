// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Autonomous.ResetTrajectory.PoseAtTime;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutonomous extends SequentialCommandGroup {
  /** Creates a new RunAutonomous. */
  DrivetrainSubsystem m_Subsystem;
  PathPlannerTrajectory trajectory;
  PathContainer pathContainer;
  public RunAutonomous(DrivetrainSubsystem m_Subsystem, PathContainer pathContainer) {
    this.m_Subsystem = m_Subsystem;
    this.pathContainer = pathContainer;
    addRequirements(m_Subsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetTrajectory(m_Subsystem, pathContainer, PoseAtTime.START),
      //RobotContainer.getFullAuto()
      new FollowTrajectory(m_Subsystem, pathContainer).withTimeout(pathContainer.getTimeout()),
      new ConditionalCommand(new ResetTrajectory(m_Subsystem, pathContainer, PoseAtTime.END),
          new InstantCommand(), pathContainer::getResetOnEnd)
    );
  }
}
