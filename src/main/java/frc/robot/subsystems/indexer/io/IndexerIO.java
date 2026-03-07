package frc.robot.subsystems.indexer.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

/**
 * The generic indexer IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface IndexerIO {
    @AutoLog
    public static abstract class IndexerInputs {
        public boolean isIndexerMotorConnected = false;
        public Angle indexerMotorPosition = Radians.of(0.0);
        public AngularVelocity indexerMotorVelocity = RadiansPerSecond.of(0.0);
        public Voltage indexerMotorAppliedVoltage = Volts.of(0.0);
        public Current indexerMotorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(IndexerInputs loggableInputs) {}

    /**
     * Directly sets the applied voltage to the indexer motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the indexer motor.
    */
    public default void setIndexerMotorVoltage(Voltage voltaeg) {}
}
