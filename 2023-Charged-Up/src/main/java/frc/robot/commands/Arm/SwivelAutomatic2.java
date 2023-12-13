// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class SwivelAutomatic2 extends CommandBase {
  /** Creates a new SwivelAutomatic2. */

  ArmSubsystem aSubsystem;
  PneumaticSubsystem pSubsystem;
  int extendGoal;
  double swivelGoal;
  PIDController levelPID;

  boolean reachedHeight = false;
  boolean swivelFirst;
  boolean auto;

  double threshold;

  Timer timer;

  public SwivelAutomatic2(ArmSubsystem aSubsystem, PneumaticSubsystem pSubsystem, double swivelGoal, int extendGoal,
      boolean swivelFirst, boolean auto) {
    this.aSubsystem = aSubsystem;
    this.pSubsystem = pSubsystem;
    this.swivelGoal = swivelGoal;
    this.extendGoal = extendGoal;
    this.swivelFirst = swivelFirst;
    this.auto = auto;

    levelPID = Constants.Arm.LevelPID;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SwivelAutomatic2(ArmSubsystem aSubsystem, PneumaticSubsystem pSubsystem, double swivelGoal, int extendGoal,
      boolean swivelFirst, boolean auto, double thresh) {
    this.aSubsystem = aSubsystem;
    this.pSubsystem = pSubsystem;
    this.swivelGoal = swivelGoal;
    this.extendGoal = extendGoal;
    this.swivelFirst = swivelFirst;
    this.auto = auto;

    levelPID = Constants.Arm.LevelPID;
    timer = new Timer();
    threshold = thresh;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();

    if (auto) {
      aSubsystem.setMaxSpeed(0.9);
      aSubsystem.setMaxExtend(0.4);
    } else {
      aSubsystem.setMaxSpeed(1);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (swivelFirst) {
      swivelFirst();
    } else {
      extendFirst();
    }

    if (RobotContainer.getArmSubsystem().getAutoWrist()) {
      if (aSubsystem.mapAbsoluteEncoder() < 4) {
        pSubsystem.closeWrist();
      } else {
        pSubsystem.openWrist();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aSubsystem.swivel(0);
    aSubsystem.extend(0);

    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (timer.get() > 2.5) {
      return true;
    }

    if (Math.abs(aSubsystem.mapAbsoluteEncoder() - swivelGoal) < threshold
        && Math.abs(aSubsystem.getExtendTicks() - extendGoal) - 5 < Constants.Arm.EXTEND_TICK_THRESHOLD) {
      aSubsystem.swivel(0);
      aSubsystem.setMove(false);
      aSubsystem.setManuelMove(false);
      return true;
    }
    return false;
  }

  private void extendFirst() {
    System.out.println(aSubsystem.mapAbsoluteEncoder() - swivelGoal);
    System.out.println("Arm Encoder thing: " + Math.abs(aSubsystem.getExtendTicks() - extendGoal));
    aSubsystem.setManuelMove(true);

    if (/* timer.get() > delay */true) {
      if (Math.abs(aSubsystem.mapAbsoluteEncoder() - swivelGoal) > threshold) {
        if (Math.abs(aSubsystem.getExtendTicks() - extendGoal) - 20 < Constants.Arm.EXTEND_TICK_THRESHOLD * 2) {
          aSubsystem.swivel(levelPID.calculate(aSubsystem.mapAbsoluteEncoder(), swivelGoal) * -1.5);
          //aSubsystem.setMove(true);

        }
      } else {
        // subsystem.swivel(0);
        reachedHeight = true;
        aSubsystem.setMove(false);
      }
    }

    aSubsystem.setExtending(true);
    // this.subsystem.setManuelMove(true);

    aSubsystem.extend(Constants.Arm.ExtendPID.calculate(aSubsystem.getExtendTicks(), extendGoal) * 1.75);
  }

  private void swivelFirst() {
    System.out.println(
        aSubsystem.mapAbsoluteEncoder() - swivelGoal + " , Extend: " + (aSubsystem.getExtendTicks() - extendGoal));
    aSubsystem.setManuelMove(true);

    if (true || false || 1 > 0 || "test".equals("test")) {
      if (Math.abs(aSubsystem.mapAbsoluteEncoder() - swivelGoal) > threshold) {
        aSubsystem.swivel(levelPID.calculate(aSubsystem.mapAbsoluteEncoder(), swivelGoal) * -2.4);
        //aSubsystem.setMove(true);

      } else {
        // subsystem.swivel(0);
        reachedHeight = true;
        aSubsystem.setMove(false);
      }
    }

    if (aSubsystem.mapAbsoluteEncoder() > 5 && aSubsystem.mapAbsoluteEncoder() < 6.5) {
      // pSubsystem.closeClaw();
      // once = false;
    } else {
      pSubsystem.enteringZone();
    }

    aSubsystem.setExtending(true);
    // this.subsystem.setManuelMove(true);
    if (Math.abs(aSubsystem.mapAbsoluteEncoder() - swivelGoal) < threshold) {
      aSubsystem.extend(Constants.Arm.ExtendPID.calculate(aSubsystem.getExtendTicks(), extendGoal) / 1.0);
    }
  }
}
