// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.RobotContainer;

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
    }
    return ret;
    }
}
