package frc.robot.subsystems.template;

import org.littletonrobotics.junction.AutoLog;

/**
 * The generic template IO interface (with support for AdvantageKit).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces
*/
public abstract interface TemplateIO {
    @AutoLog
    public static abstract class TemplateInputs {}

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(TemplateInputs loggableInputs) {}
}
