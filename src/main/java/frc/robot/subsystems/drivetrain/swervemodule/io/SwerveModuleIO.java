package frc.robot.subsystems.drivetrain.swervemodule.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

/**
 * The generic swerve module IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface SwerveModuleIO {
    @AutoLog
    public static abstract class SwerveModuleInputs {
        public boolean isDriveMotorConnected = false;
        public Angle driveMotorPosition = Radians.of(0.0);
        public AngularVelocity driveMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration driveMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage driveMotorAppliedVoltage = Volts.of(0.0);
        public Current driveMotorStatorCurrent = Amps.of(0.0);

        public boolean isAzimuthMotorConnected = false;
        public Angle azimuthMotorPosition = Radians.of(0.0);
        public AngularVelocity azimuthMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration azimuthMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage azimuthMotorAppliedVoltage = Volts.of(0.0);
        public Current azimuthMotorStatorCurrent = Amps.of(0.0);

        public boolean isAzimuthEncoderConnected = false;
        public Voltage azimuthEncoderSupplyVoltage = Volts.of(0.0);
        public Angle azimuthEncoderPosition = Radians.of(0.0);
        public AngularVelocity azimuthEncoderVelocity = RadiansPerSecond.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(SwerveModuleInputs loggableInputs) {}

    /**
     * Sets the drive motor's onboard velocity setpoint from a given drive wheel velocity.
     * 
     * @param velocity The desired linear velocity of the drive wheel at the contact point.
    */
    public default void setWheelVelocity(LinearVelocity velocity) {}

    /**
     *  Sets the azimuth motor's onboard position setpoint from a given drive wheel angle.
     * 
     * @param azimuth The desired angle of the drive wheel relative to the encoder's zero.
    */
    public default void setWheelAzimuth(Angle azimuth) {}

    /**
     * Directly sets the applied voltage to the drive motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the drive motor.
    */
    public default void setDriveMotorVoltage(Voltage voltage) {}

    /**
     * Directly sets the applied voltage to the azimuth motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the azimuth motor.
    */
    public default void setAzimuthMotorVoltage(Voltage voltage) {}
}
