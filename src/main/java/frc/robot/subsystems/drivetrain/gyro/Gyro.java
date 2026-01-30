package frc.robot.subsystems.drivetrain.gyro;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.io.GyroInputsAutoLogged;

/**
 * A gyro pseudo-subsystem that contains basic helper methods and allows for defined periodic updates.
*/
public final class Gyro {
    private final GyroIO m_gyroIO;
    private final GyroInputsAutoLogged m_gyroInputs = new GyroInputsAutoLogged();

    public Gyro(GyroIO gyroIO) {
        m_gyroIO = gyroIO;
    }

    public final void periodic() {
        m_gyroIO.updateInputs(m_gyroInputs);
        Logger.processInputs("DrivetrainSubsystem/Gyro", m_gyroInputs);
    }

    public final void zero() {
        m_gyroIO.zeroYaw();
    }

    public final Angle getAngle() {
        return m_gyroInputs.yaw;
    }

    public final Rotation2d getAngleAsRotation() {
        return Rotation2d.fromRadians(getAngle().in(Radians));
    }

    public final AngularVelocity getAngularVelocity() {
        return m_gyroInputs.yawRate;
    }
}
