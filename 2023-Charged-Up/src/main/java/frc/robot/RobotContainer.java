// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.AutoLookup;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoniterPnuematics;
import frc.robot.commands.SetLevelMode;
import frc.robot.commands.SwivelAutomatic2;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.Alignments.AlignAT;
import frc.robot.commands.Alignments.AlignCharge;
import frc.robot.commands.Alignments.AlignGamepiece;
import frc.robot.commands.Alignments.AlignOnceAT;
import frc.robot.commands.Alignments.Level;
import frc.robot.commands.Arm.ArmLevel;
import frc.robot.commands.Arm.CloseClaw;
import frc.robot.commands.Arm.Extend;
import frc.robot.commands.Arm.ExtendAutomatic;
import frc.robot.commands.Arm.ExtendLevel;
import frc.robot.commands.Arm.OpenClaw;
import frc.robot.commands.Arm.ResetArmEncoder;
import frc.robot.commands.Arm.Swivel;
import frc.robot.commands.Arm.SwivelAutomatic;
import frc.robot.commands.Arm.SwivelEnd;
import frc.robot.commands.Arm.ZeroExtendEncoder;
import frc.robot.commands.Autonomous.AutoLookup;
import frc.robot.commands.Intake.SpinIntake;
import frc.robot.commands.Pneumatics.ClawPneumatic;
import frc.robot.commands.Pneumatics.IntakePneumatic;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.commands.PathContainer;
//import frc.robot.commands.RunAutonomous;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // private final XboxController m_controller = new XboxController(0);
  // JoystickButton buttonA = new JoystickButton(m_controller, 1);
  static PathPlannerTrajectory practicePath = PathPlanner.loadPath("practice", new PathConstraints(1, 0.50));

  CommandXboxController driveController = new CommandXboxController(0);
  CommandXboxController armController = new CommandXboxController(1);
  PhotonCamera cam = new PhotonCamera("front_camera");

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  static PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
  static ArmSubsystem armSubsystem = new ArmSubsystem();
  static Vision vision = new Vision();
  static Limelight limelightSubsystem = new Limelight();

  @SuppressWarnings("rawtypes")
  private static SendableChooser m_autoChooser;

  @SuppressWarnings("rawtypes")
  private static SendableChooser m_allianceChooser;

  private static SendableCameraWrapper m_camera;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_autoChooser = new SendableChooser<>();
    m_autoChooser.setDefaultOption("Nothing", null);

    //m_autoChooser.addOption("practice", AutoLookup.getAuto("practice"));
    m_autoChooser.addOption("Middle", AutoLookup.getAuto("Middle"));
    m_autoChooser.addOption("Cable", AutoLookup.getAuto("Cable"));
    m_autoChooser.addOption("Right", AutoLookup.getAuto("Right"));

    m_autoChooser.addOption("Tester", AutoLookup.getAuto("Tester"));

    SmartDashboard.putData(m_autoChooser);

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(driveController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    armSubsystem.setDefaultCommand(new ArmLevel(armSubsystem));
    pneumaticSubsystem.setDefaultCommand(new MoniterPnuematics(pneumaticSubsystem));

    m_drivetrainSubsystem.resetOdometry();
    // Configure the button bindings

    CameraServer.startAutomaticCapture();

    configureButtonBindings();
  }

  static SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_drivetrainSubsystem.getKinematics(),
      m_drivetrainSubsystem.getGyroscopeRotation(),

      new SwerveModulePosition[] {
          m_drivetrainSubsystem.frontLeftPos(),
          m_drivetrainSubsystem.frontRightPos(),
          m_drivetrainSubsystem.backLeftPos(),
          m_drivetrainSubsystem.backRightPos()
      }, new Pose2d(m_drivetrainSubsystem.getPose().getX(), m_drivetrainSubsystem.getPose().getY(), new Rotation2d()));

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope

    // Zero Gyro (ON PRESS)
    driveController.start().whileTrue(new ZeroGyro(m_drivetrainSubsystem));

    // Spin Intake (WHILE HELD)
    driveController.rightBumper().whileTrue(new SpinIntake(intakeSubsystem, Constants.Intake.SPIN_SPEED));
    driveController.rightBumper().whileFalse(new SpinIntake(intakeSubsystem, 0));

    // Grab Intake (TOGGLE)
    driveController.leftBumper().whileTrue(new IntakePneumatic(pneumaticSubsystem));

    // driveController.leftTrigger(0.1).whileTrue(new AlignAT(limelightSubsystem,
    // m_drivetrainSubsystem, 2.05, true));
    driveController.leftTrigger(0.1).onTrue(
        new SequentialCommandGroup(
          new OpenClaw(pneumaticSubsystem),
            new AlignGamepiece(armSubsystem, m_drivetrainSubsystem, vision),
            new CloseClaw(pneumaticSubsystem)
            ));
    driveController.rightTrigger(0.1).whileTrue(new Level(m_drivetrainSubsystem, 1));

    // driveController.y().onTrue(new SetLevelMode(armSubsystem, "cone"));
    // driveController.x().onTrue(new SetLevelMode(armSubsystem, "cube"));
    // driveController.b().onTrue(new SetLevelMode(armSubsystem, "none"));

    // extend (WHILE HELD)
    armController.rightBumper().whileTrue(new Extend(armSubsystem, Constants.Arm.EXTEND_SPEED));
    armController.rightBumper().onFalse(new ExtendLevel(armSubsystem));

    // Retract (WHILE HELD)
    armController.leftBumper().whileTrue(new Extend(armSubsystem, -Constants.Arm.EXTEND_SPEED));
    armController.leftBumper().onFalse(new ExtendLevel(armSubsystem));

    // Swivel Up (WHILE HELD)
    armController.leftTrigger(0.1)
        .whileTrue(new Swivel(armSubsystem, pneumaticSubsystem, Constants.Arm.SWIVEL_SPEED, armController, 2));
    armController.leftTrigger(0.1).onFalse(new SwivelEnd(armSubsystem));

    // Swivel Down (WHILE HELD)
    armController.rightTrigger(0.1)
        .whileTrue(new Swivel(armSubsystem, pneumaticSubsystem, -Constants.Arm.SWIVEL_SPEED, armController, 3));
    armController.rightTrigger(0.1).onFalse(new SwivelEnd(armSubsystem));

    // Claw Grab (TOGGLE)
    armController.x().whileTrue(new ClawPneumatic(pneumaticSubsystem));

    // Automated Swivel ()
    armController.y().onTrue(new SwivelAutomatic2(armSubsystem, pneumaticSubsystem, Constants.Arm.TICKS_TO_HIGH,
        Constants.Arm.TICKS_TO_FAR_EXTEND, true, false));
    armController.b().onTrue(new SwivelAutomatic2(armSubsystem, pneumaticSubsystem, Constants.Arm.TICKS_TO_MID,
        Constants.Arm.TICKS_TO_CLOSE_EXTEND, true, false));
    armController.a()
        .onTrue(new SwivelAutomatic2(armSubsystem, pneumaticSubsystem, Constants.Arm.TICKS_TO_BOTTOM, 100, true, false));
    armController.povUp()
        .onTrue(new SwivelAutomatic2(armSubsystem, pneumaticSubsystem, Constants.Arm.TICKS_TO_SUBSTATION, 10, true, false));
    armController.povUp().onFalse(new SwivelEnd(armSubsystem));

    armController.povRight()
        .onTrue(new SwivelAutomatic2(armSubsystem, pneumaticSubsystem, Constants.Arm.TICKS_TO_ZERO, 5, false, false));
    armController.povRight().onFalse(new SwivelEnd(armSubsystem));

    armController.povDown()
        .onTrue(new SwivelAutomatic2(armSubsystem, pneumaticSubsystem, Constants.Arm.TICKS_TO_FlOOR, 65000, true, false));
    armController.povDown().onFalse(new SwivelEnd(armSubsystem));

    armController.y().onFalse(new SwivelEnd(armSubsystem));
    armController.b().onFalse(new SwivelEnd(armSubsystem));
    armController.a().onFalse(new SwivelEnd(armSubsystem));
    // Reset Arm Encoder (ON PRESS)
    armController.start().whileTrue(new ZeroExtendEncoder(armSubsystem));
  }

  public void test() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  static SwerveDriveKinematics m_kinematics = m_drivetrainSubsystem.getKinematics();

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (Command) m_autoChooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }

  public static DrivetrainSubsystem getDriveSubsystem() {
    return m_drivetrainSubsystem;
  }

  public static Vision getVision() {
    return vision;
  }

  public static ArmSubsystem getArmSubsystem() {
    return armSubsystem;
  }

  public static PneumaticSubsystem getPneumaticSubsystem() {
    return pneumaticSubsystem;
  }

  public static Limelight getLimelightSubsystem() {
    return limelightSubsystem;
  }
}