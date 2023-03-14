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
    public static final int CLAW_CHANNEL = 6;
    //public static final int CLAW_REVERSE_CHANNEL = 7;

    public static final int INTAKE_FORWARD_CHANNEL = 5;
    public static final int INTAKE_REVERSE_CHANNEL = 4;

    public static final int INTAKE_DEPLOY_FORWARD_CHANNEL = 3;
    public static final int INTAKE_DEPLOY_REVERSE_CHANNEL = 2;
  }

  public static class Controller {
    public static final int DRIVE_CONTROLLER_ID = 0;
    public static final int ARM_CONTROLLER_ID = 1;
  }

  public static class Intake {
    public static final int MOTOR_SPIN_ID = 11;
    public static final double SPIN_SPEED = 0.90;
  }

  public static class Arm {

    public static final double SWIVEL_ABSOLUTE_OFFSET = -0.07030065175751629 * 0;

    public static final double LEVEL_CONSTANT = 0.95;

    public static final int SWIVEL_MOTOR_ID = 9;
    public static final int EXTENTION_MOTOR_ID = 10;

    public static final int ENCODER_INPUT_ID = 0;
    public static final int ENCODER_OUTPUT_ID = 1;

    public static final double SWIVEL_SPEED = 0.40;
    public static final double EXTEND_SPEED = 0.45;

    public static final double TICKS_TO_HIGH = 2.70;
    public static final double TICKS_TO_MID = 3.15;
    public static final double TICKS_TO_BOTTOM = 5.0;
    public static final double TICKS_TO_SUBSTATION = 2.90;
    public static final double TICKS_TO_ZERO = 5.65;
    public static final double TICKS_TO_FlOOR = 4.75;

    public static final int EXTEND_FLOOR = 75000;

    public static final int TICKS_TO_FAR_EXTEND = 65000;
    public static final int TICKS_TO_CLOSE_EXTEND = 10000;

    public static final double TICK_THRESHOLD = .35;
    public static final double EXTEND_TICK_THRESHOLD = 1000;

    public static final int MAX_TICKS = 400;
    //public static final int MIN_TICKS = -5;

    public static final PIDController LevelPID = new PIDController(0.23, 0, 0);
    public static final PIDController ExtendPID = new PIDController(0.01, 0, 0);

    public static final double KS = 0.32889;
    public static final double KA = 2.4516;
    public static final double KG = 3.3777;
    public static final double KV = 6.3053;

    public static final double EXTEND_KS = 0.032812;
    public static final double EXTEND_KA = 0.00040743;
    public static final double EXTEND_KG = -0.27321;
    public static final double EXTEND_KV = 0.019186;
  }

  public static class Drivetrain {
    public static final int GYROSCOPE_ID = 1;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_STEER_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(291.45355224609375);
    //-336.357422

    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_STEER_ID = 4;
    public static final int FRONT_RIGHT_ENCODER_ID = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(344.5374755859375);
    //-352.6172

    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_STEER_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 4;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(296.630859375);
    //-289.160156

    public static final int BACK_RIGHT_DRIVE_ID = 5;
    public static final int BACK_RIGHT_STEER_ID = 6;
    public static final int BACK_RIGHT_ENCODER_ID = 3;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(289.86053466796875);
    //-296.1914

    //L-R between center of wheels
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4572;

    //F-B between center of wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4572;

    public static final Translation2d FRONT_LEFT_TRANS_FROM_CENTER = new Translation2d(0.2832, 0.2832);
    public static final Translation2d FRONT_RIGHT_TRANS_FROM_CENTER = new Translation2d(0.2832, -0.2832);
    public static final Translation2d BACK_LEFT_TRANS_FROM_CENTER = new Translation2d(-0.2832, 0.2832);
    public static final Translation2d BACK_RIGHT_TRANS_FROM_CENTER = new Translation2d(-0.2832, -0.2832);

    public static final class auto {

      private static final double MAX_ANG_VEL_RAD_AUTO = 8 * Math.PI;
      //public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(subsystems.H_Auto.MAX_ANGULAR_VELOCITY, subsystems.H_Auto.MAX_ANGULAR_ACCELERATION);
      public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(6380.0/60.0, 10*Math.PI);

      
      public static final PIDController X_PID_CONTROLLER = new PIDController(.2357, 0.0, 0.19);
      
      public static final PIDController Y_PID_CONTROLLER = new PIDController(.03709, 0.0, 0.07);
      
      public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(0.415, 0, 0.205,
              ROT_PROFILE);
      // DRIVING DEFAULT IS 5
      public static final double LINEAR_VELOCITY_DEFAULT = 0;
// BEST SO FAR .1 .1 3.4

    }
  }
}
