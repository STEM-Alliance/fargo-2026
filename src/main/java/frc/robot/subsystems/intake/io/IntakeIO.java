package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

/**
 * The generic intake IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface IntakeIO {
    @AutoLog
    public static abstract class IntakeInputs {
        public double compressorPressure = 0.0;
        public Voltage compressorVoltage = Volts.of(0.0);
        public Current compressorCurrent = Amps.of(0.0);

        public boolean isIntakeExtended = false;

        public boolean isIntakeMotorConnected = false;
        public Angle intakeMotorPosition = Radians.of(0.0);
        public AngularVelocity intakeMotorVelocity = RadiansPerSecond.of(0.0);
        public Voltage intakeMotorAppliedVoltage = Volts.of(0.0);
        public Current intakeMotorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(IntakeInputs loggableInputs) {}

    /**
     * Sets the position of the intake's pnuematics.
     * 
     * @param extended Whether or not the pnuematics should be forward.
    */
    public default void setIntakeExtended(boolean extended) {}

    /**
     * Directly sets the applied voltage to the intake motor.
     * 
     * @param voltage The voltage to apply to the intake motor.
    */
    public default void setIntakeMotorVoltage(Voltage voltage) {}
}
