// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Delay;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ExtendRetract;
import frc.robot.commands.Go;
import frc.robot.commands.SetPipeline;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.Alignments.AlignAT;
import frc.robot.commands.Alignments.AlignCharge;
import frc.robot.commands.Alignments.AlignConeNode;
import frc.robot.commands.Alignments.AlignGamepiece;
import frc.robot.commands.Alignments.GOANDROTATE3;
import frc.robot.commands.Alignments.GoAndRotate;
import frc.robot.commands.Alignments.GoAndRotate2;
import frc.robot.commands.Alignments.Level;
import frc.robot.commands.Alignments.Rotate;
import frc.robot.commands.Alignments.SnapCube;
import frc.robot.commands.Arm.CloseClaw;
import frc.robot.commands.Arm.Extend;
import frc.robot.commands.Arm.ExtendAutomatic;
import frc.robot.commands.Arm.ExtendLevel;
import frc.robot.commands.Arm.NewSwivel;
import frc.robot.commands.Arm.OpenClaw;
import frc.robot.commands.Arm.Swivel;
import frc.robot.commands.Arm.SwivelAutomatic;
import frc.robot.commands.Arm.SwivelAutomatic2;
import frc.robot.commands.Arm.ToggleAutoWrist;
import frc.robot.commands.Arm.ZeroExtendEncoder;
import frc.robot.commands.Pneumatics.ClawPneumatic;
import frc.robot.commands.Pneumatics.ToggleWrist;

/** Add your docs here. */
public class AutoLookup {

        public static AutoRoutine getAuto(String name) {
                AutoRoutine ret;

                switch (name) {
                        default:
                                // testing for path planner
                        case "ONLY PLACE":
                                ret = new AutoRoutine(
                                                new ParallelCommandGroup(
                                                                new ExtendLevel(RobotContainer.getArmSubsystem()),
                                                                new NewSwivel(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                0.1))

                                );
                                break;
                        // Place one and balance Charge station
                        case "Middle Potatoes":
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
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(0.6),
                                                                                new SwivelAutomatic2(RobotContainer
                                                                                                .getArmSubsystem(),
                                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                                Constants.Arm.TICKS_TO_ZERO
                                                                                                                + 0.4,
                                                                                                0,
                                                                                                false,
                                                                                                true))),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 2.05,
                                                                true),
                                                new Level(RobotContainer.getDriveSubsystem(), 0));
                                break;
                        // Tester for auto level

                        case "MIDDLE 1.5 TENDERS WITH POTATOES":
                                ret = new AutoRoutine(
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SequentialCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH + .25,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 2,
                                                                                true, true, 0.4)),
                                                new WaitCommand(0.2),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new WaitCommand(0.15),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                                new AlignAT(RobotContainer.getLimelightSubsystem(),
                                                                RobotContainer.getDriveSubsystem(), 1.0, true),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_FlOOR + 0.5,
                                                                                Constants.Arm.EXTEND_FLOOR - 12, false,
                                                                                true, 0.05),
                                                                new GoAndRotate(-245, 0, 0,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0.1, 90, 1.4624)),
                                                new Rotate(179, RobotContainer.getDriveSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                
                                                // new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                /*
                                                 * new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                 * RobotContainer.getDriveSubsystem(),
                                                 * RobotContainer.getVision(), 1),
                                                 */

                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new WaitCommand(0.25),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ParallelCommandGroup(
                                                                new ExtendRetract(RobotContainer.getArmSubsystem(),
                                                                                -.6),
                                                                new GOANDROTATE3(-145, 0,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0, 15)),
                                                                
                                                                
                                                new Level(RobotContainer.getDriveSubsystem(), 180));
                                break;
                        case "BLUE LEFT 1.5 TENDERS WITH POTATOES":
                                ret = new AutoRoutine(
                                /*
                                 * new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                 * new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                 * new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                 * new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                 * new SequentialCommandGroup(
                                 * new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getPneumaticSubsystem(),
                                 * Constants.Arm.TICKS_TO_HIGH,
                                 * Constants.Arm.TICKS_TO_FAR_EXTEND - 15,
                                 * true, true, 0.4),
                                 * new SequentialCommandGroup(
                                 * new WaitCommand(0.3),
                                 * new AlignAT(RobotContainer
                                 * .getLimelightSubsystem(),
                                 * RobotContainer.getDriveSubsystem(),
                                 * 0.5, false))),
                                 * new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new WaitCommand(0.15),
                                 * new AlignAT(RobotContainer.getLimelightSubsystem(),
                                 * RobotContainer.getDriveSubsystem(), 1.15, true),
                                 * new SequentialCommandGroup(
                                 * new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getPneumaticSubsystem(),
                                 * Constants.Arm.TICKS_TO_FlOOR,
                                 * Constants.Arm.EXTEND_FLOOR, false,
                                 * true, 0.15),
                                 * new GoAndRotate(-245, 0, 0,
                                 * RobotContainer.getDriveSubsystem(),
                                 * 0.1, 90, 1.26)),
                                 * new Rotate(179, RobotContainer.getDriveSubsystem()),
                                 * new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                 * new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getDriveSubsystem(),
                                 * RobotContainer.getVision(), 1),
                                 * new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new ParallelCommandGroup(
                                 * new ExtendRetract(RobotContainer.getArmSubsystem(),
                                 * -.6),
                                 * new GoAndRotate2(160, 180,
                                 * RobotContainer.getDriveSubsystem(),
                                 * -0.075, 5)),
                                 * new Level(RobotContainer.getDriveSubsystem(), 180)
                                 */
                                );
                                break;
                        case "RED RIGHT 1.5 TENDERS WITH POTATOES":
                                ret = new AutoRoutine(
                                /*
                                 * new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                 * new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                 * new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                 * new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                 * new SequentialCommandGroup(
                                 * new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getPneumaticSubsystem(),
                                 * Constants.Arm.TICKS_TO_HIGH,
                                 * Constants.Arm.TICKS_TO_FAR_EXTEND - 15,
                                 * true, true, 0.4),
                                 * new SequentialCommandGroup(
                                 * new WaitCommand(0.3),
                                 * new AlignAT(RobotContainer
                                 * .getLimelightSubsystem(),
                                 * RobotContainer.getDriveSubsystem(),
                                 * 0.5, false))),
                                 * new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new WaitCommand(0.15),
                                 * new AlignAT(RobotContainer.getLimelightSubsystem(),
                                 * RobotContainer.getDriveSubsystem(), 1.95, true),
                                 * new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                 * new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                 * new SequentialCommandGroup(
                                 * new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getPneumaticSubsystem(),
                                 * Constants.Arm.TICKS_TO_FlOOR,
                                 * Constants.Arm.EXTEND_FLOOR - 5, false,
                                 * true, 0.15),
                                 * new GoAndRotate(-245, 0, 0,
                                 * RobotContainer.getDriveSubsystem(),
                                 * 0.1, 90, 1.26)),
                                 * new Rotate(179, RobotContainer.getDriveSubsystem()),
                                 * new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                 * new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getDriveSubsystem(),
                                 * RobotContainer.getVision(), 1),
                                 * new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new ParallelCommandGroup(
                                 * new ExtendRetract(RobotContainer.getArmSubsystem(),
                                 * -.6),
                                 * new GoAndRotate2(160, 180,
                                 * RobotContainer.getDriveSubsystem(),
                                 * -0.075, 5)),
                                 * new Level(RobotContainer.getDriveSubsystem(), 180)
                                 */
                                );
                                break;
                        case "RED RIGHT 2 TENDERS":
                                ret = new AutoRoutine(
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.4),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH - 0.2,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND - 2,
                                                                                true, true, 0.3),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(1.41),
                                                                                //new AlignConeNode(RobotContainer.getDriveSubsystem(), RobotContainer.getPoseSubsystem(), RobotContainer.getVision(), 0).withTimeout(.5),
                                                                                new DriveForward(RobotContainer
                                                                                                .getDriveSubsystem(),
                                                                                                0.3,
                                                                                                RobotContainer.getVision()),
                                                                                new WaitCommand(0.7),
                                                                                new ToggleWrist(RobotContainer
                                                                                                .getPneumaticSubsystem()),
                                                                                new DriveForward(RobotContainer
                                                                                                .getDriveSubsystem(), 0,
                                                                                                RobotContainer.getVision()))),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                // new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new WaitCommand(0.25),
                                                // new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate(-255, 75, 170,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0.1, 5, 1.85),
                                                                new SequentialCommandGroup(
                                                                                new ExtendRetract(RobotContainer
                                                                                                .getArmSubsystem(),
                                                                                                -.3),
                                                                                new SwivelAutomatic2(RobotContainer
                                                                                                .getArmSubsystem(),
                                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                                Constants.Arm.TICKS_TO_FlOOR
                                                                                                                + 0.5,
                                                                                                Constants.Arm.EXTEND_FLOOR
                                                                                                                - 5,
                                                                                                false,
                                                                                                true, 0.05))),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new WaitCommand(0.5),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                RobotContainer.getVision(), 0).withTimeout(1.5),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ParallelCommandGroup(
                                
                                                                new GoAndRotate2(-245, 170,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                -0.175, 5),
                                                                               
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH - 0.2,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND - 4,
                                                                                true, true, 0.2)),
                                                new WaitCommand(0.25),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new Rotate(20, RobotContainer.getDriveSubsystem()),
                                                new WaitCommand(0.25),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new SnapCube(RobotContainer.getDriveSubsystem(), RobotContainer.getPoseSubsystem(), RobotContainer.getVision(), 0).withTimeout(1.5),
                                                // new Delay(0.3),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH - 0.3, 0,
                                                                true, true, 0.15));

                                break;
                        case "BLUE LEFT 2 TENDERS":
                        ret = new AutoRoutine(
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.4),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH - 0.2,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND - 2,
                                                                                true, true, 0.3),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(1.41),
                                                                                //new AlignConeNode(RobotContainer.getDriveSubsystem(), RobotContainer.getPoseSubsystem(), RobotContainer.getVision(), 0).withTimeout(.5),
                                                                                new DriveForward(RobotContainer
                                                                                                .getDriveSubsystem(),
                                                                                                0.3,
                                                                                                RobotContainer.getVision()),
                                                                                new WaitCommand(0.7),
                                                                                new ToggleWrist(RobotContainer
                                                                                                .getPneumaticSubsystem()),
                                                                                new DriveForward(RobotContainer
                                                                                                .getDriveSubsystem(), 0,
                                                                                                RobotContainer.getVision()))),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                // new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new WaitCommand(0.25),
                                                // new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new ParallelCommandGroup(
                                                                new GoAndRotate(-255, -75, -170,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                -0.1, 5, 1.85),
                                                                new SequentialCommandGroup(
                                                                                new ExtendRetract(RobotContainer
                                                                                                .getArmSubsystem(),
                                                                                                -.3),
                                                                                new SwivelAutomatic2(RobotContainer
                                                                                                .getArmSubsystem(),
                                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                                Constants.Arm.TICKS_TO_FlOOR
                                                                                                                + 0.5,
                                                                                                Constants.Arm.EXTEND_FLOOR
                                                                                                                - 5,
                                                                                                false,
                                                                                                true, 0.05))),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new WaitCommand(0.5),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getDriveSubsystem(),
                                                                RobotContainer.getVision(), 0).withTimeout(1.5),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ParallelCommandGroup(
                                
                                                                new GoAndRotate2(-245, -170,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0.175, 5),
                                                                               
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH - 0.2,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND - 4,
                                                                                true, true, 0.2)),
                                                new WaitCommand(0.25),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new Rotate(-20, RobotContainer.getDriveSubsystem()),
                                                new WaitCommand(0.25),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new SnapCube(RobotContainer.getDriveSubsystem(), RobotContainer.getPoseSubsystem(), RobotContainer.getVision(), 0).withTimeout(1.5),
                                                // new Delay(0.3),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH - 0.3, 0,
                                                                true, true, 0.15));

                                break;
                        // 2 Cube WITH cable protector
                        case "RED CABLE 2 TENDERS":
                                ret = new AutoRoutine(
                                        new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_HIGH - 0.2,
                                                                                Constants.Arm.TICKS_TO_FAR_EXTEND+2,
                                                                                true, true, 0.3),
                                                                new SequentialCommandGroup(
                                                                                new WaitCommand(1.6),
                                                                                //new AlignConeNode(RobotContainer.getDriveSubsystem(), RobotContainer.getPoseSubsystem(), RobotContainer.getVision(), 0).withTimeout(.5),
                                                                                new DriveForward(RobotContainer
                                                                                                .getDriveSubsystem(),
                                                                                                0.5,
                                                                                                RobotContainer.getVision()),
                                                                                new WaitCommand(0.6),
                                                                                new ToggleWrist(RobotContainer
                                                                                                .getPneumaticSubsystem()),
                                                                                new DriveForward(RobotContainer
                                                                                                .getDriveSubsystem(), 0,
                                                                                                RobotContainer.getVision()))),
                                                new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                                 new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new WaitCommand(0.25),
                                               // new SequentialCommandGroup(
                                                 //               new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                   //                             RobotContainer.getPneumaticSubsystem(),
                                                     //                           Constants.Arm.TICKS_TO_HIGH + .3,
                                                       //                         Constants.Arm.TICKS_TO_FAR_EXTEND + 2,
                                                         //                       true, true, 0.4)),
                                           //     new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                             //   new WaitCommand(0.1),
                                               // new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                       //         new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                         //       new AlignAT(RobotContainer.getLimelightSubsystem(),
                                           //                     RobotContainer.getDriveSubsystem(), 1.0, true),
                                                new ParallelCommandGroup(
                                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                                RobotContainer.getPneumaticSubsystem(),
                                                                                Constants.Arm.TICKS_TO_FlOOR + 0.5,
                                                                                Constants.Arm.EXTEND_FLOOR - 12, false,
                                                                                true, 0.05),
                                                                new GoAndRotate(-245, 0, 0,
                                                                                RobotContainer.getDriveSubsystem(),
                                                                                0, 90, 1.4624)),
                                                new Rotate(179, RobotContainer.getDriveSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                                new AlignGamepiece(RobotContainer.getArmSubsystem(), RobotContainer.getDriveSubsystem(), RobotContainer.getVision(), 1).withTimeout(2)
                                                //new ToggleAutoWrist(RobotContainer.getArmSubsystem())

                                /*
                                 * new AlignGamepiece(RobotContainer.getArmSubsystem(),
                                 * RobotContainer.getDriveSubsystem(),
                                 * RobotContainer.getVision(), 1),
                                 */

                                /*
                                 * new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                 * new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                 * new GoAndRotate(0, 0, 180, RobotContainer.getDriveSubsystem(), 0, 25, 0)
                                 */
                                );
                                break;
                        case "BLUE CABLE 2 TENDERS":
                        ret = new AutoRoutine(
                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                        new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                        new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                        new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                        new ExtendRetract(RobotContainer.getArmSubsystem(), -0.2),
                                        new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                        new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                        new ParallelCommandGroup(
                                                        new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                        RobotContainer.getPneumaticSubsystem(),
                                                                        Constants.Arm.TICKS_TO_HIGH - 0.2,
                                                                        Constants.Arm.TICKS_TO_FAR_EXTEND+2,
                                                                        true, true, 0.3),
                                                        new SequentialCommandGroup(
                                                                        new WaitCommand(1.6),
                                                                        //new AlignConeNode(RobotContainer.getDriveSubsystem(), RobotContainer.getPoseSubsystem(), RobotContainer.getVision(), 0).withTimeout(.5),
                                                                        new DriveForward(RobotContainer
                                                                                        .getDriveSubsystem(),
                                                                                        0.5,
                                                                                        RobotContainer.getVision()),
                                                                        new WaitCommand(0.6),
                                                                        new ToggleWrist(RobotContainer
                                                                                        .getPneumaticSubsystem()),
                                                                        new DriveForward(RobotContainer
                                                                                        .getDriveSubsystem(), 0,
                                                                                        RobotContainer.getVision()))),
                                        new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                                         new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                        new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                        new WaitCommand(0.25),
                                       // new SequentialCommandGroup(
                                         //               new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                           //                             RobotContainer.getPneumaticSubsystem(),
                                             //                           Constants.Arm.TICKS_TO_HIGH + .3,
                                               //                         Constants.Arm.TICKS_TO_FAR_EXTEND + 2,
                                                 //                       true, true, 0.4)),
                                   //     new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                     //   new WaitCommand(0.1),
                                       // new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                               //         new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                 //       new AlignAT(RobotContainer.getLimelightSubsystem(),
                                   //                     RobotContainer.getDriveSubsystem(), 1.0, true),
                                        new ParallelCommandGroup(
                                                        new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                        RobotContainer.getPneumaticSubsystem(),
                                                                        Constants.Arm.TICKS_TO_FlOOR + 0.5,
                                                                        Constants.Arm.EXTEND_FLOOR - 12, false,
                                                                        true, 0.05),
                                                        new GoAndRotate(-245, 0, 0,
                                                                        RobotContainer.getDriveSubsystem(),
                                                                        0, 90, 1.4624)),
                                        new Rotate(179, RobotContainer.getDriveSubsystem()),
                                        new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                        new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                        new OpenClaw(RobotContainer.getPneumaticSubsystem()),
                                        new AlignGamepiece(RobotContainer.getArmSubsystem(), RobotContainer.getDriveSubsystem(), RobotContainer.getVision(), 1).withTimeout(2)
                                        //new ToggleAutoWrist(RobotContainer.getArmSubsystem())

                        /*
                         * new AlignGamepiece(RobotContainer.getArmSubsystem(),
                         * RobotContainer.getDriveSubsystem(),
                         * RobotContainer.getVision(), 1),
                         */

                        /*
                         * new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                         * new ToggleAutoWrist(RobotContainer.getArmSubsystem()),
                         * new GoAndRotate(0, 0, 180, RobotContainer.getDriveSubsystem(), 0, 25, 0)
                         */
                        );
                                break;

                        // Tester auto
                        case "Tester":
                                ret = new AutoRoutine(
                                                new SetPipeline(RobotContainer.getLimelightSubsystem(), "april"),
                                                new CloseClaw(RobotContainer.getPneumaticSubsystem()),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new ZeroGyro(RobotContainer.getDriveSubsystem()),
                                                new ExtendRetract(RobotContainer.getArmSubsystem(), -0.4),
                                                new ZeroExtendEncoder(RobotContainer.getArmSubsystem()),
                                                new SwivelAutomatic2(RobotContainer.getArmSubsystem(),
                                                                RobotContainer.getPneumaticSubsystem(),
                                                                Constants.Arm.TICKS_TO_HIGH - 0.1,
                                                                Constants.Arm.TICKS_TO_FAR_EXTEND + 17,
                                                                true, true, 0.3),
                                                // new ToggleWrist(RobotContainer.getPneumaticSubsystem()),
                                                new OpenClaw(RobotContainer.getPneumaticSubsystem()));

                                break;
                }
                return ret;
        }
}
