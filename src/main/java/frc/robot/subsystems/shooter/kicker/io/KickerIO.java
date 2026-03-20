package frc.robot.subsystems.shooter.kicker.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

/**
 * The generic kicker IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface KickerIO {
    @AutoLog
    public static abstract class KickerInputs {
        public boolean isKickerMotorConnected = false;
        public Angle kickerMotorPosition = Radians.of(0.0);
        public AngularVelocity kickerMotorVelocity = RadiansPerSecond.of(0.0);
        public Voltage kickerMotorAppliedVoltage = Volts.of(0.0);
        public Current kickerMotorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(KickerInputs loggableInputs) {}

    /**
     * Directly sets the applied voltage to the kicker motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the kicker motor.
    */
    public default void setKickerMotorVoltage(Voltage voltage) {}
}
