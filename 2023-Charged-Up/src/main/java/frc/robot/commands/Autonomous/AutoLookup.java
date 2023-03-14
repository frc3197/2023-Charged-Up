// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Delay;
import frc.robot.commands.ExtendRetract;
import frc.robot.commands.Go;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.Alignments.AlignAT;
import frc.robot.commands.Alignments.AlignCharge;
import frc.robot.commands.Alignments.AlignGamepiece;
import frc.robot.commands.Alignments.GoAndRotate;
import frc.robot.commands.Alignments.GoAndRotate2;
import frc.robot.commands.Alignments.Level;
import frc.robot.commands.Alignments.Rotate;
import frc.robot.commands.Arm.CloseClaw;
import frc.robot.commands.Arm.Extend;
import frc.robot.commands.Arm.ExtendAutomatic;
import frc.robot.commands.Arm.OpenClaw;
import frc.robot.commands.Arm.SwivelAutomatic;
import frc.robot.commands.Arm.SwivelAutomatic2;
import frc.robot.commands.Arm.ZeroExtendEncoder;
import frc.robot.commands.Pneumatics.ClawPneumatic;

/** Add your docs here. */
public class AutoLookup {

        public static AutoRoutine getAuto(String name) {
                AutoRoutine ret;

                switch (name) {
                        default:
                                // testing for path planner
                        case "practice":
                                ret = new AutoRoutine(
                                                new RunAutonomous(RobotContainer.getDriveSubsystem(),
                                                                PathLookup.getContainer("practice")));
                                break;
                        // Place one and balance Charge station
                        case "Middle":
                                ret = new AutoRoutine(
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SequentialCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND,
                                                                                true, true),
                                                                new SequentialCommandGroup(
                                                                                new Delay(0.3),
                                                                                new AlignAT(RobotContainer
                                                                                                .getLimelightSubsystem(),
                                                                                                RobotContainer.getDriveSubsystem(),
                                                                                                0.5, false))),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.45),
                                                new ParallelCommandGroup(
                                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                1.5, true),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO + 0.4, 0,
                                                                                false,
                                                                                true)),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 2.05,
                                                                true),
                                                new Level(RobotContainer.getDriveSubsystem(), 1));
                                break;
                        // Tester for auto level
                        case "Charge":
                                ret = new AutoRoutine(
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 0.7,
                                                                false),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 2.15,
                                                                true),
                                                new Level(RobotContainer.getDriveSubsystem(), 1));
                                break;
                        // 2 cube auto WITHOUT cable protector
                        case "Right_RED":
                                ret = new AutoRoutine(
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH - 0.105,
                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 22500,
                                                                true, true),
                                                /*
                                                 * new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                 * RobotContainer.getDriveSubsystem(), 0.5, false),
                                                 */
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate(-235, 180,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                -0.1, 5),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 0, false,
                                                                                true)),
                                                // new Delay(0.25),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_FlOOR,
                                                                Constants.Arm.EXTEND_FLOOR, true, false),

                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),

                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                RobotContainer.getVision(), 0),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate2(-220, 180,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0.075, 5),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 5, false,
                                                                                true)),
                                                new AlignAT(RobotContainer
                                                                .getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                0.7, false),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH,
                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 33000,
                                                                true, true),
                                               // new Delay(0.3),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()));

                                break;
                        case "Right_BLUE":
                                ret = new AutoRoutine(
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH,
                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 3000,
                                                                true, true),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 0.5, false),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate(-235, 180,
                                                                                RobotContainer.getDriveSubsystem(), 0.1,
                                                                                5),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 0, false,
                                                                                true)),
                                                // new Delay(0.25),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_FlOOR,
                                                                Constants.Arm.EXTEND_FLOOR, true, false),

                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),

                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                RobotContainer.getVision(), 0),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate2(-195, 180,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                -0.075, 5),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 5, false,
                                                                                true)),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_MID,
                                                                                Constants.Arm.TICKS_TO_CLOSE_EXTEND,
                                                                                true, true),
                                                                new SequentialCommandGroup(
                                                                                new Delay(0.3),
                                                                                new AlignAT(RobotContainer
                                                                                                .getLimelightSubsystem(),
                                                                                                RobotContainer.getDriveSubsystem(),
                                                                                                0.5, false))),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()));
                                break;
                        // 2 Cube WITH cable protector
                        case "Cable_RED":
                                ret = new AutoRoutine(
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH,
                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 1000,
                                                                true, true),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 0.5, false),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate2(-260, 180,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0.15, 20),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 0, false,
                                                                                true)),
                                                // new Delay(0.25),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_FlOOR,
                                                                Constants.Arm.EXTEND_FLOOR, true, false),

                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),

                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                RobotContainer.getVision(), 0),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate(-230, 180,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                -0.075, 20),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 5, false,
                                                                                true)),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_MID,
                                                                                Constants.Arm.TICKS_TO_CLOSE_EXTEND,
                                                                                true, true),
                                                                new SequentialCommandGroup(
                                                                                new Delay(0.3),
                                                                                new AlignAT(RobotContainer
                                                                                                .getLimelightSubsystem(),
                                                                                                RobotContainer.getDriveSubsystem(),
                                                                                                0.5, false))),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()));
                                break;
                        case "Cable_BLUE":
                                ret = new AutoRoutine(
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH,
                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 1000,
                                                                true, true),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 0.5, false),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate2(-260, 180,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                -0.15, 40),
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_ZERO, 0, false,
                                                                                true)),
                                                // new Delay(0.25),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_FlOOR,
                                                                Constants.Arm.EXTEND_FLOOR, true, false),

                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),

                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new Delay(0.3),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                RobotContainer.getVision(), 0),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem())
                                /*
                                 * new ParallelCommandGroup(
                                 * new GoAndRotate(-230, 180,
                                 * RobotContainer.getDriveSubsystem(), 0.075, 30),
                                 * new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getPneumaticSubsystem(),
                                 * Constants.Arm.TICKS_TO_ZERO, 5, false,
                                 * true)),
                                 * new ParallelCommandGroup(
                                 * new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getPneumaticSubsystem(),
                                 * Constants.Arm.TICKS_TO_MID,
                                 * Constants.Arm.TICKS_TO_CLOSE_EXTEND,
                                 * true, true),
                                 * new SequentialCommandGroup(
                                 * new Delay(0.3),
                                 * new AlignAT(RobotContainer
                                 * .getLimelightSubsystem(),
                                 * RobotContainer.getDriveSubsystem(),
                                 * 0.5, false))),
                                 * new OpenClaw(RobotContainer.getPneumaticSubsystem())
                                 */);
                                break;

                        // Tester auto
                        case "Tester":
                                ret = new AutoRoutine(
                                                new GoAndRotate(-270, 180, RobotContainer.getDriveSubsystem(), 0, 5));
                                break;
                }
                return ret;
        }
}
