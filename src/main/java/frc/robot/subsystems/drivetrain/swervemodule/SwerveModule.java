package frc.robot.subsystems.drivetrain.swervemodule;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.kDriveMotorToWheelFactor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO.SwerveModuleInputs;

/**
 * A swerve module pseudo-subsystem that contains basic helper methods and allows for defined periodic updates.
*/
public final class SwerveModule {
    private final String m_swerveModuleName;
    private final SwerveModuleIO m_swerveModuleIO;
    private final SwerveModuleInputsAutoLogged m_swerveModuleInputs = new SwerveModuleInputsAutoLogged();

    public SwerveModule(String swerveModuleName, SwerveModuleIO swerveModuleIO) {
        m_swerveModuleName = swerveModuleName;
        m_swerveModuleIO = swerveModuleIO;
    }

    public final void periodic() {
        m_swerveModuleIO.updateInputs(m_swerveModuleInputs);
        Logger.processInputs("DrivetrainSubsystem/SwerveModule" + m_swerveModuleName, m_swerveModuleInputs);
    }

    public final SwerveModuleInputs getInputs() {
        return m_swerveModuleInputs;
    }

    public final void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentWheelAzimuth = getWheelAzimuthAsRotation();

        desiredState.optimize(currentWheelAzimuth);
        desiredState.cosineScale(currentWheelAzimuth);

        m_swerveModuleIO.setWheelVelocity(MetersPerSecond.of(desiredState.speedMetersPerSecond));
        m_swerveModuleIO.setWheelAzimuth(Radians.of(desiredState.angle.getRadians()));
    }

    public final void setDriveFFAccel(double gain) {
        m_swerveModuleIO.setDriveFFAccel(gain);
    }

    public final void setWheelVelocity(LinearVelocity velocity) {
        m_swerveModuleIO.setWheelVelocity(velocity);
    }

    public final void setWheelAzimuth(Angle azimuth) {
        m_swerveModuleIO.setWheelAzimuth(azimuth);
    }

    public final void setDriveMotorVoltage(Voltage voltage) {
        m_swerveModuleIO.setDriveMotorVoltage(voltage);
    }

    public final void setAzimuthMotorVoltage(Voltage voltage) {
        m_swerveModuleIO.setAzimuthMotorVoltage(voltage);
    }

    public final SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getWheelPosition(), getWheelAzimuthAsRotation());
    }

    public final SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), getWheelAzimuthAsRotation());
    }

    public final Distance getWheelPosition() {
        return Meters.of(m_swerveModuleInputs.driveMotorPosition.in(Rotations) * kDriveMotorToWheelFactor);
    }

    public final LinearVelocity getWheelVelocity() {
        return MetersPerSecond.of(
            m_swerveModuleInputs.driveMotorVelocity.in(RotationsPerSecond) * kDriveMotorToWheelFactor
        );
    }

    public final Angle getWheelAzimuth() {
        return m_swerveModuleInputs.azimuthEncoderPosition;
    }

    public final Rotation2d getWheelAzimuthAsRotation() {
        return Rotation2d.fromRadians(getWheelAzimuth().in(Radians));
    }
}
