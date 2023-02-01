// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class Arm {
    public static final int SWIVEL_MOTOR_ID = 0;
    public static final int EXTENTION_MOTOR_ID = 0;

    public static final double SWIVEL_SPEED = 0.25;
    public static final double EXTEND_SPEED = 0.25;

    public static final int SOFT_LIM_MAX = 50000;
    public static final int SOFT_LIM_MIN = 0;
  }

  public static class Drivetrain {
    public static final int FRONT_LEFT_DRIVE_ID = 0;
    public static final int FRONT_LEFT_STEER_ID = 0;
    public static final int FRONT_LEFT_ENCODER_ID = 0;

    public static final int FRONT_RIGHT_DRIVE_ID = 0;
    public static final int FRONT_RIGHT_STEER_ID = 0;
    public static final int FRONT_RIGHT_ENCODER_ID = 0;


  }
}
