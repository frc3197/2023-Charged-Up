package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        //System.out.println(m_drivetrainSubsystem.getGyro());
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
         //System.out.println("Test");

        /*System.out.println(m_drivetrainSubsystem.getFrontLeftMod().getDriveVelocity());
        System.out.println(m_drivetrainSubsystem.getFrontRightMod().getDriveVelocity());
        System.out.println(m_drivetrainSubsystem.getBackLeftMod().getDriveVelocity());
        System.out.println(m_drivetrainSubsystem.getBackRightMod().getDriveVelocity());*/
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public DoubleSupplier getTransXSupplier()
    {
        return m_translationXSupplier;
    }

    public DoubleSupplier getTransYSupplier()
    {
        return m_translationYSupplier;
    }

    public DoubleSupplier getRotationSupplier()
    {
        return m_rotationSupplier;
    }
}