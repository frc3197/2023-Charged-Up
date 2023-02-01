// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Intake {
    public static final int MOTOR_DEPLOY_ID = 0;

    public static final double DEPLOY_SPEED = 0.25;
  }

  public static class Arm {
    public static final int SWIVEL_MOTOR_ID = 9;
    public static final int EXTENTION_MOTOR_ID = 10;

    public static final int ENCODER_INPUT_ID = 0;
    public static final int ENCODER_OUTPUT_ID = 1;

    public static final double SWIVEL_SPEED = 0.25;
    public static final double EXTEND_SPEED = 0.25;

    public static final int TICKS_TO_HIGH = 50000;
    public static final int TICKS_TO_MID = 25000;
    public static final int TICKS_TO_BOTTOM = 1000;

    public static final int TICKS_TO_FAR = 5000;
    public static final int TICKS_TO_CLOSE = 2500;

    public static final int TICK_THRESHOLD = 10;

    public static final PIDController LevelPID = new PIDController(0, 0, 0);
  }

  public static class Drivetrain {
    public static final int GYROSCOPE_ID = 1;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_STEER_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 1;

    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_STEER_ID = 4;
    public static final int FRONT_RIGHT_ENCODER_ID = 2;

    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_STEER_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 4;

    public static final int BACK_RIGHT_DRIVE_ID = 5;
    public static final int BACK_RIGHT_STEER_ID = 6;
    public static final int BACK_RIGHT_ENCODER_ID = 3;
  }
}
