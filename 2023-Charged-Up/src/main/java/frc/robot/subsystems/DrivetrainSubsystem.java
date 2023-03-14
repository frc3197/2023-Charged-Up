// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

public class DrivetrainSubsystem extends SubsystemBase {

        Translation2d frontLeftPos = new Translation2d(0.2832, 0.2832);
        Translation2d frontRightPos = new Translation2d(0.2832, -0.2832);
        Translation2d backLeftPos = new Translation2d(-0.2832, 0.2832);
        Translation2d backRightPos = new Translation2d(-0.2832, -0.2832);

        CANCoder frontLeftSteerEncoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_ENCODER_ID);
        CANCoder frontRightSteerEncoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_ENCODER_ID);
        CANCoder backLeftSteerEncoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_ENCODER_ID);
        CANCoder backRightSteerEncoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_ENCODER_ID);

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;
        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-Constants.Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -Constants.Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        // FIXME Remove if you are using a Pigeon
        // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
        private final com.ctre.phoenix.sensors.Pigeon2 m_pigeon2 = new com.ctre.phoenix.sensors.Pigeon2(
                        Constants.Drivetrain.GYROSCOPE_ID);
        // FIXME Uncomment if you are using a NavX
        // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
        // connected over MXP

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        WPI_TalonFX driveMotor1 = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID);
        WPI_TalonFX driveMotor2 = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID);
        WPI_TalonFX driveMotor3 = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_ID);
        WPI_TalonFX driveMotor4 = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID);

        private SwerveModuleState[] m_desiredStates;

        SwerveDrivePoseEstimator poseEstimation;

        Pose2d robotPose = new Pose2d();
        private double offset = 0;

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                driveMotor1 = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID);
                driveMotor2 = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_ID);
                driveMotor3 = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_ID);
                driveMotor4 = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_ID);

                driveMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                driveMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                driveMotor3.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                driveMotor4.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

                driveMotor1.setSelectedSensorPosition(0);
                driveMotor2.setSelectedSensorPosition(0);
                driveMotor3.setSelectedSensorPosition(0);
                driveMotor4.setSelectedSensorPosition(0);

                // Your module has two Falcon 500s on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createNeo(...)
                // Your module has two NEOs on it. One for steering and one for driving.
                //
                // Mk3SwerveModuleHelper.createFalcon500Neo(...)
                // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
                // and the NEO is for steering.
                //
                // Mk3SwerveModuleHelper.createNeoFalcon500(...)
                // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
                // Falcon 500 is for steering.
                //
                // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
                // class.

                // By default we will use Falcon 500s in standard configuration. But if you use
                // a different configuration or motors
                // you MUST change it. If you do not, your code will crash on startup.
                // FIXME Setup motor configuration

                m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                // This is the ID of the drive motor
                                Constants.Drivetrain.FRONT_LEFT_DRIVE_ID,
                                // This is the ID of the steer motor
                                Constants.Drivetrain.FRONT_LEFT_STEER_ID,
                                // This is the ID of the steer encoder
                                Constants.Drivetrain.FRONT_LEFT_ENCODER_ID,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                Constants.Drivetrain.FRONT_RIGHT_DRIVE_ID,
                                Constants.Drivetrain.FRONT_RIGHT_STEER_ID,
                                Constants.Drivetrain.FRONT_RIGHT_ENCODER_ID,
                                Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                Constants.Drivetrain.BACK_LEFT_DRIVE_ID,
                                Constants.Drivetrain.BACK_LEFT_STEER_ID,
                                Constants.Drivetrain.BACK_LEFT_ENCODER_ID,
                                Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                                Constants.Drivetrain.BACK_RIGHT_DRIVE_ID,
                                Constants.Drivetrain.BACK_RIGHT_STEER_ID,
                                Constants.Drivetrain.BACK_RIGHT_ENCODER_ID,
                                Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET);

                m_desiredStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

                //SwerveModulePosition[] positions = {m_frontLeftModule.};

                poseEstimation = new SwerveDrivePoseEstimator(m_kinematics, getGyro(), 
                new SwerveModulePosition[]{frontLeftPos(), frontRightPos(), backLeftPos(), backRightPos()},
                new Pose2d());
        }

        /*public double getSwerveDriveEstimation() {
                
        }*/
                
        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                // FIXME Remove if you are using a Pigeon
                // m_pigeon.setFusedHeading(0.0);
                m_pigeon2.setYaw(0);
                // resetOdometry();

                // FIXME Uncomment if you are using a NavX
                // m_navx.zeroYaw();
        }

        public Rotation2d getGyroscopeRotation() {
                // FIXME Remove if you are using a Pigeon
                // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
                return Rotation2d.fromDegrees(m_pigeon2.getYaw());

                // FIXME Uncomment if you are using a NavX
                // if (m_navx.isMagnetometerCalibrated()) {
                // // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }
                //
                // // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public double getPitch() {
                return m_pigeon2.getPitch();
        }

        public double getRoll() {
                return m_pigeon2.getRoll();
        }

        public double getYaw() {
                return m_pigeon2.getYaw();
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
                m_desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_desiredStates;
                // SwerveDriveKinematics.normalizeWheelSpeeds(states,
                // MAX_VELOCITY_METERS_PER_SECOND);
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                Rotation2d gyroAngle = getGyro();

                // Update the pose
                robotPose = RobotContainer.getOdometry().update(gyroAngle,
                                new SwerveModulePosition[] {
                                                frontLeftPos(), frontRightPos(),
                                                backLeftPos(), backRightPos()
                                });

                updateOdometry();
        }

        public void updateOdometry() {
                m_odometry.update(Rotation2d.fromDegrees(getGyroscopeRotation().getDegrees()),
                                new SwerveModulePosition[] { frontLeftPos(), frontRightPos(), backLeftPos(),
                                                backRightPos() });
        }

        public SwerveModulePosition getPosition(int moduleNum) {

                double driveEncoder1pos = driveMotor1.getSelectedSensorPosition();
                double driveEncoder2pos = driveMotor2.getSelectedSensorPosition();
                double driveEncoder3pos = driveMotor3.getSelectedSensorPosition();
                double driveEncoder4pos = driveMotor4.getSelectedSensorPosition();

                if (moduleNum == 1) {
                        return new SwerveModulePosition(
                                        driveEncoder1pos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
                                                        / ((6.75) * 4096),
                                        new Rotation2d(frontRightSteerEncoder.getAbsolutePosition() * Math.PI / 180));
                } else if (moduleNum == 2) {
                        return new SwerveModulePosition(
                                        driveEncoder2pos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
                                                        / ((6.75) * 4096),
                                        new Rotation2d(backRightSteerEncoder.getAbsolutePosition() * Math.PI / 180));
                } else if (moduleNum == 3) {
                        return new SwerveModulePosition(
                                        driveEncoder3pos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
                                                        / ((6.75) * 4096),
                                        new Rotation2d(backLeftSteerEncoder.getAbsolutePosition() * Math.PI / 180));
                } else {
                        return new SwerveModulePosition(
                                        driveEncoder4pos * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
                                                        / ((6.75) * 4096),
                                        new Rotation2d(frontLeftSteerEncoder.getAbsolutePosition() * Math.PI / 180));
                }
        }

        public SwerveDriveKinematics getKinematics() {
                // MASON MCMANUS WROTE THIS
                return m_kinematics;
        }

        public ChassisSpeeds getCurrentSpeed() {

                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                // SwerveDriveKinematics.normalizeWheelSpeeds(states,
                // MAX_VELOCITY_METERS_PER_SECOND);
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                ChassisSpeeds speed;
                speed = m_kinematics.toChassisSpeeds(states);
                return speed;
        }

        SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(),
                        new SwerveModulePosition[] { frontLeftPos(), frontRightPos(), backLeftPos(), backRightPos() });

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void resetOdometry() {
                zeroGyroscope();
                RobotContainer.getOdometry().resetPosition(getGyroscopeRotation(),
                                new SwerveModulePosition[] { frontLeftPos(), frontRightPos(), backLeftPos(),
                                                frontRightPos() },
                                getPose());
                // zeroGyroscope();
        }

        public void resetOdometry(Pose2d resetPos) {
                zeroGyroscope();
                RobotContainer.getOdometry().resetPosition(getGyro(),
                                new SwerveModulePosition[] { frontLeftPos(), frontRightPos(), backLeftPos(),
                                                backRightPos() },
                                resetPos);
        }

        public void updateStates(SwerveModuleState[] states) {

                m_desiredStates = states;
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
                // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getCurrentSpeed(),
                // MAX_VELOCITY_METERS_PER_SECOND, MAX_VELOCITY_METERS_PER_SECOND,
                // MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
                m_kinematics.toChassisSpeeds(desiredStates);
        }

        public void setOffset(double offset) {
                this.offset = offset;
        }

        public double getOffset() {
                return offset;
        }

        public SwerveModulePosition frontLeftPos() {
                return getPosition(4);
        }

        public SwerveModulePosition frontRightPos() {
                return getPosition(1);
        }

        public SwerveModulePosition backLeftPos() {
                return getPosition(3);
        }

        public SwerveModulePosition backRightPos() {
                return getPosition(2);
        }

        public Rotation2d getGyro() {
                return getGyroscopeRotation();
        }

        public SwerveModule getFrontLeftMod() {
                return m_frontLeftModule;
        }

        public SwerveModule getFrontRightMod() {
                return m_frontRightModule;
        }

        public SwerveModule getBackLeftMod() {
                return m_backLeftModule;
        }

        public SwerveModule getBackRightMod() {
                return m_backRightModule;
        }

        public Pose2d getPoseEstimation()
        {
                
                poseEstimation.update(getGyro(), new SwerveModulePosition[] {frontLeftPos(), frontRightPos(), backLeftPos(), backRightPos()});
                return poseEstimation.getEstimatedPosition();
        }
}