// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class Limelight {
    public static final int APRIL_TAG_PIPELINE_INDEX = 0;
    public static final int RETRO_REFLECTIVE_TAPE_PIPELINE_INDEX = 1;

    public static final String limelightName = "limelight";

  }

  public static class Pneumatics {
    public static final int CLAW_FORWARD_CHANNEL = 6;
    public static final int CLAW_REVERSE_CHANNEL = 7;
    public static final int INTAKE_FORWARD_CHANNEL = 4;
    public static final int INTAKE_REVERSE_CHANNEL = 5;
  }

  public static class Controller {
    public static final int DRIVE_CONTROLLER_ID = 0;
    public static final int ARM_CONTROLLER_ID = 1;
  }

  public static class Intake {
    public static final int MOTOR_DEPLOY_ID = 0;
    public static final int MOTOR_SPIN_ID = 11;

    public static final double DEPLOY_SPEED = 0.25;
    public static final double SPIN_SPEED = 0.90;
  }

  public static class Arm {
    public static final int SWIVEL_MOTOR_ID = 9;
    public static final int EXTENTION_MOTOR_ID = 10;

    public static final int ENCODER_INPUT_ID = 0;
    public static final int ENCODER_OUTPUT_ID = 1;

    public static final double SWIVEL_SPEED = 0.15;
    public static final double EXTEND_SPEED = 0.25;

    public static final int TICKS_TO_HIGH = 300;
    public static final int TICKS_TO_MID = 200;
    public static final int TICKS_TO_BOTTOM = 50;

    public static final int TICKS_TO_FAR = 8000;
    public static final int TICKS_TO_CLOSE = 2500;

    public static final int TICK_THRESHOLD = 2;

    public static final PIDController LevelPID = new PIDController(0.3, 0, 0);
  }

  public static class Drivetrain {
    public static final int GYROSCOPE_ID = 1;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_STEER_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(291.17889404296875);

    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_STEER_ID = 4;
    public static final int FRONT_RIGHT_ENCODER_ID = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(280.951904296875);

    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_STEER_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 4;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(296.01287841796875);

    public static final int BACK_RIGHT_DRIVE_ID = 5;
    public static final int BACK_RIGHT_STEER_ID = 6;
    public static final int BACK_RIGHT_ENCODER_ID = 3;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(288.62457275390625);

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4572;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4572;

    public static final Translation2d FRONT_LEFT_TRANS_FROM_CENTER = new Translation2d(0.2832, 0.2832);
    public static final Translation2d FRONT_RIGHT_TRANS_FROM_CENTER = new Translation2d(0.2832, -0.2832);
    public static final Translation2d BACK_LEFT_TRANS_FROM_CENTER = new Translation2d(-0.2832, 0.2832);
    public static final Translation2d BACK_RIGHT_TRANS_FROM_CENTER = new Translation2d(-0.2832, -0.2832);

    public static final class auto {

      private static final double MAX_ANG_VEL_RAD_AUTO = 8 * Math.PI;
      //public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(subsystems.H_Auto.MAX_ANGULAR_VELOCITY, subsystems.H_Auto.MAX_ANGULAR_ACCELERATION);
      public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(6380.0/60.0, 10*Math.PI);

      
      public static final PIDController X_PID_CONTROLLER = new PIDController(.2358, 0, 0.18);
      
      public static final PIDController Y_PID_CONTROLLER = new PIDController(.00709, 0.03, 0.07);
      
      public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(0.415, 0, 0.205,
              ROT_PROFILE);
      // DRIVING DEFAULT IS 5
      public static final double LINEAR_VELOCITY_DEFAULT = 0;
// BEST SO FAR .1 .1 3.4

    }

  }
}
