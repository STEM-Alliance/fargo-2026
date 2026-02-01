package frc.robot.subsystems.shooter.kicker.io;

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
 * The generic kicker IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface KickerIO {
    @AutoLog
    public static abstract class KickerInputs {
        public boolean isMotorConnected = false;
        public Angle motorPosition = Radians.of(0.0);
        public AngularVelocity motorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration motorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage motorAppliedVoltage = Volts.of(0.0);
        public Current motorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(KickerInputs loggableInputs) {}

    /**
     * Sets the kicker motor's onboard velocity setpoint.
     * 
     * @param velocity The desired angular velocity of the kicker motor.
    */
    public default void setMotorVelocity(AngularVelocity velocity) {}

    /**
     * Directly sets the applied voltage to the kicker motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the kicker motor.
    */
    public default void setMotorVoltage(Voltage voltaeg) {}
}
