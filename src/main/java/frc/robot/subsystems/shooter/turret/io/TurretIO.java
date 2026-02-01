package frc.robot.subsystems.shooter.turret.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * The generic turret IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface TurretIO {
    @AutoLog
    public static abstract class TurretInputs {
        public boolean isAzimuthMotorConnected = false;
        public Angle azimuthMotorPosition = Radians.of(0.0);
        public AngularVelocity azimuthMotorVelociy = RadiansPerSecond.of(0.0);
        public AngularAcceleration azimuthMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage azimuthMotorAppliedVoltage = Volts.of(0.0);
        public Current azimuthMotorStatorCurrent = Amps.of(0.0);

        public boolean isAzimuthEncoderConnected = false;
        public Angle azimuthEncoderPosition = Radians.of(0.0);
        public AngularVelocity azimuthEncoderVelocity = RadiansPerSecond.of(0.0);
        public Voltage azimuthEncoderSupplyVoltage = Volts.of(0.0);

        public boolean isHoodMotorConnected = false;
        public Angle hoodMotorPosition = Radians.of(0.0);
        public AngularVelocity hoodMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration hoodMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage hoodMotorAppliedVoltage = Volts.of(0.0);
        public Current hoodMotorStatorCurrent = Amps.of(0.0);

        public boolean isHoodHomingSwitchPressed = false;
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(TurretInputs loggableInputs) {}

    /**
     * Sets the azimuth motor's onboard position setpoint from a given turret angle.
     * 
     * @param azimuth The desired angle of the turret relative to the encoder's zero.
    */
    public default void setTurretAzimuth(Angle azimuth) {}

    /**
     * Resets the azimuth motor's encoder position to the supplied position.
     * 
     * @param position The position to reset the motor's encoder to, relative to itself.
    */
    public default void setAzimuthMotorPosition(Angle position) {}

    /**
     * Sets the hood motor's onboard position setpoint from a given launch angle.
     * 
     * @param angle The desired launch angle of the hood relative to the horizontal.
    */
    public default void setHoodAngle(Angle angle) {}

    /**
     * Directly sets the applied voltage to the hood motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the hood motor.
    */
    public default void setHoodMotorVoltage(Voltage voltage) {}

    /**
     * Resets the hood motor's encoder position to the supplied position.
     * 
     * @param position The position to reset the motor's encoder to, relative to itself.
    */
    public default void setHoodMotorPosition(Angle position) {}
}
