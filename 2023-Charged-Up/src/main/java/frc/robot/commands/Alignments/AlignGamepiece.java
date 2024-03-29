// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignments;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Vision;

public class AlignGamepiece extends CommandBase {
  
  //Variables
  ArmSubsystem armSubsystem;
  DrivetrainSubsystem driveSubsystem;
  Vision vision;
  Timer timer;
  double pitch;
  double yaw;
  double maxPitch = 1.5;
  double maxYaw = 0.25;

  double xSpeed;
  double ySpeed;
  double yawSpeed;

  int pipeline;

  private int counter = 0;

  private double timeout = 5.0;

  public AlignGamepiece(ArmSubsystem aSub, DrivetrainSubsystem dSub, Vision vSub, int pipeline) {
    //Constructor
    armSubsystem = aSub;
    driveSubsystem = dSub;
    vision = vSub;
    this.pipeline = pipeline;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timer start sequence
    timer.stop();
    timer.reset();
    timer.start();

    vision.setPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(driveSubsystem.getYaw());
    //num targets
    if (vision.hasTarget() && (vision.getPitch()) > 5.5) {
       counter = 0;
    } else {
      counter++;
    }

    if ((vision.getPitch()) < 4.5) {
      counter++;
    } else {
      //counter = 0;
    }

    pitch = driveSubsystem.getPitch();
    yaw = driveSubsystem.getYaw();
    //speed regulation
    double pitchSpeed = pitch / 10;
    if (pitchSpeed > maxPitch) {
      pitchSpeed = maxPitch;
    }
    if (pitchSpeed < maxPitch * -1) {
      pitchSpeed = maxPitch * -1;
    }

    yawSpeed = yaw / -6;
    //sets speeds
    /*xSpeed = Math.cos(Units.degreesToRadians(botYaw)) * pitchSpeed;
    ySpeed = Math.sin(Units.degreesToRadians(botYaw)) * pitchSpeed;
    if (botYaw > 90 && botYaw < 270) {
      xSpeed *= -1;
    }*/
    //System.out.println(xSpeed + ", " + ySpeed);
    ySpeed = pitchSpeed;
    driveSubsystem.drive(new ChassisSpeeds(pitchSpeed, 0, yawSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops driving and timer
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(vision.getCancel()) {
       return true;
    }

    //Ends command if exceeds timeout
    if (timer.get() > timeout) {
      return true;
    }

    //Ends command if bot is within threshold
    System.out.println(xSpeed);
    return  Math.abs(ySpeed) < 0.2 && Math.abs(yawSpeed) < 0.2 && counter > 20;
  }
}
