// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.CloseClaw;
import frc.robot.commands.Arm.ExtendAutomatic;
import frc.robot.commands.Arm.OpenClaw;
import frc.robot.commands.Arm.SwivelAutomatic;
import frc.robot.commands.Arm.ZeroExtendEncoder;
import frc.robot.commands.Pneumatics.ClawPneumatic;

/** Add your docs here. */
public class AutoLookup {

    public static AutoRoutine getAuto(String name)
    {
        AutoRoutine ret;
    
    switch (name)
    {
        default:
        case "practice":
            ret = new AutoRoutine(
                new RunAutonomous(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("practice"))
            );
            break;
        case "1PLACEB":
            ret = new AutoRoutine(
                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                new ParallelCommandGroup(new SwivelAutomatic(RobotContainer.getArmSubsystem(), RobotContainer.getPneumaticSubsystem(), "high"), new ExtendAutomatic(RobotContainer.getArmSubsystem(), "far")),
                new OpenClaw(RobotContainer.getPneumaticSubsystem())
                //new RunAutonomous(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("1.1"))
                /*new RunAutonomous(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("1.1")),
                new RunAutonomous(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("1.2")),
                new RunAutonomous(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("1.3.PLACE"))*/
            );
            break;
        case "Tester":
        ret = new AutoRoutine(
            new RunAutonomous(RobotContainer.getDriveSubsystem(), PathLookup.getContainer("1.1B"))
        );
        break;
    }
    return ret;
    }
}
