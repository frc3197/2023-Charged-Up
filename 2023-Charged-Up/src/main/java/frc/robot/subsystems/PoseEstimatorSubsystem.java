// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private DrivetrainSubsystem driveSubsystem;
  private Limelight limelightSubsystem;

  private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05,
      Units.degreesToRadians(5));
  private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5,
      Units.degreesToRadians(5));

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  public PoseEstimatorSubsystem(DrivetrainSubsystem dSub, Limelight lSub) {
    this.driveSubsystem = dSub;
    this.limelightSubsystem = lSub;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator = new SwerveDrivePoseEstimator(
        driveSubsystem.getKinematics(),
        driveSubsystem.getGyroscopeRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    //System.out.println(driveSubsystem.getPose());
    if(limelightSubsystem.getTargets() && limelightSubsystem.getArea() > 0.120) {
      //double[] visionArray = limelightSubsystem.getTargetSpace();
      //double resultTimestamp = limelightSubsystem.getLatency();
      double[] botPose = limelightSubsystem.getBotPoseBlue();
      Pose3d actualPose = new Pose3d(botPose[0], botPose[1], botPose[2], new Rotation3d(Units.degreesToRadians(botPose[3]), Units.degreesToRadians(botPose[4]), Units.degreesToRadians(botPose[5])));
      
      //System.out.println(Timer.getFPGATimestamp() - (botPose[6]/1000.0));
      //Pose3d camPose = new Pose3d(new Translation3d(5, new Rotation3d(visionArray[3], visionArray[4], visionArray[5])), new Rotation3d());
      
      /*Transform3d camToTarget = target.getBestCameraToTarget();
      Pose3d camPose = targetPose.transformBy(camToTarget.inverse());*/

      poseEstimator.addVisionMeasurement(actualPose.toPose2d(), Timer.getFPGATimestamp() - (botPose[6]/1000.0));
    }

    poseEstimator.update(
      driveSubsystem.getGyroscopeRotation(),
      new SwerveModulePosition[] { driveSubsystem.frontLeftPos(), driveSubsystem.frontRightPos(), driveSubsystem.backLeftPos(),
        driveSubsystem.backRightPos() });

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose() {
    poseEstimator.resetPosition(driveSubsystem.getGyroscopeRotation(),
    driveSubsystem.getModulePositions(), getCurrentPose());
  }
}
