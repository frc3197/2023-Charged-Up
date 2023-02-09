// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignRRT extends CommandBase {
  /** Creates a new AlignRRT. */

  DrivetrainSubsystem subsystem;
  double[] pose;

  private NetworkTable table;

  public AlignRRT(DrivetrainSubsystem subsystem) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //0-2 are offset in meters, goes x, y, z, rest are rotations I assume
    pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

  }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
