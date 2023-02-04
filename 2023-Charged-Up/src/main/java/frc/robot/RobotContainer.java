// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Arm.Extend;
import frc.robot.commands.Arm.Swivel;
import frc.robot.commands.Autonomous.AutoLookup;
import frc.robot.commands.Intake.SpinIntake;
import frc.robot.commands.Pneumatics.ClawPneumatic;
import frc.robot.commands.Pneumatics.IntakePneumatic;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  CommandXboxController drive_controller = new CommandXboxController(Constants.Controller.DRIVE_CONTROLLER_ID);
  CommandXboxController arm_controller = new CommandXboxController(Constants.Controller.ARM_CONTROLLER_ID);
  //JoystickButton bButton = new JoystickButton(controller, 2);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  static DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();
  IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  PneumaticSubsystem m_PneumaticSubsystem = new PneumaticSubsystem();

  @SuppressWarnings("rawtypes")
  private static SendableChooser m_autoChooser;

  @SuppressWarnings("rawtypes")
  private static SendableChooser m_allianceChooser;

  public RobotContainer() {
    // Configure the trigger bindings

    m_autoChooser = new SendableChooser<>();

    m_autoChooser.setDefaultOption("Nothing", null);
    m_autoChooser.addOption("practice", AutoLookup.getAuto("practice"));

    SmartDashboard.putData(m_autoChooser);

    m_DrivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_DrivetrainSubsystem, 
      () -> -modifyAxis(drive_controller.getLeftY())*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -modifyAxis(drive_controller.getLeftY())*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
      () -> -modifyAxis(drive_controller.getRightX())*DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_DrivetrainSubsystem.resetOdometry();
    
    configureBindings();
  }

  private void configureBindings() {

    //Swivel --- (WHILE HELD)
    arm_controller.b().whileTrue(new Swivel(m_ArmSubsystem, "mid", Constants.Arm.SWIVEL_SPEED));
    arm_controller.b().whileFalse(new Swivel(m_ArmSubsystem, null, 0));

    // Swivel --- (WHILE HELD)
    arm_controller.x().whileTrue(new Swivel(m_ArmSubsystem, "high", -Constants.Arm.SWIVEL_SPEED));
    arm_controller.x().whileFalse(new Swivel(m_ArmSubsystem, null, 0));

    //Extend Arm (WHILE HELD)
    arm_controller.y().whileTrue(new Extend(m_ArmSubsystem, Constants.Arm.EXTEND_SPEED));
    arm_controller.y().whileFalse(new Extend(m_ArmSubsystem, 0));

    // Retract Arm (WHILE HELD)
    arm_controller.a().whileTrue(new Extend(m_ArmSubsystem, -Constants.Arm.EXTEND_SPEED));
    arm_controller.a().whileFalse(new Extend(m_ArmSubsystem, 0));

    //Intake spin (WHILE HELD)
    drive_controller.a().whileTrue(new SpinIntake(m_IntakeSubsystem, Constants.Intake.SPIN_SPEED));
    drive_controller.a().whileFalse(new SpinIntake(m_IntakeSubsystem, 0));

    //Intake Cramp (TOGGLE)
    drive_controller.x().whileTrue(new IntakePneumatic(m_PneumaticSubsystem));

    //Claw Grab (TOGGLE)
    drive_controller.y().whileTrue(new ClawPneumatic(m_PneumaticSubsystem));
  }

  static SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_DrivetrainSubsystem.getKinematics(), m_DrivetrainSubsystem.getGyroscopeRotation(), 
  
  new SwerveModulePosition[]
  {
    m_DrivetrainSubsystem.frontLeftPos(),
    m_DrivetrainSubsystem.frontRightPos(),
    m_DrivetrainSubsystem.backLeftPos(),
    m_DrivetrainSubsystem.backRightPos()
  }, new Pose2d(m_DrivetrainSubsystem.getPose().getX(), m_DrivetrainSubsystem.getPose().getY(), new Rotation2d()));

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return (Command) m_autoChooser.getSelected();
  }

  public double getTrigger()
  {
    return drive_controller.getRightTriggerAxis();
  }

  public static SwerveDriveOdometry getOdometry()
  {
    return m_odometry;
  }

  public static DrivetrainSubsystem getDriveSubsystem()
  {
    return m_DrivetrainSubsystem;
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
}
