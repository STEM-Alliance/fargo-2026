package frc.robot.subsystems.drivetrain.gyro.io;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * The generic gyro IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface GyroIO {
    @AutoLog
    public static abstract class GyroInputs {
        public boolean isConnected = false;
        public Voltage supplyVoltage = Volts.of(0.0);
        public Angle pitch = Radians.of(0.0);
        public AngularVelocity pitchRate = RadiansPerSecond.of(0.0);
        public Angle roll = Radians.of(0.0);
        public AngularVelocity rollRate = RadiansPerSecond.of(0.0);
        public Angle yaw = Radians.of(0.0);
        public AngularVelocity yawRate = RadiansPerSecond.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(GyroInputs loggableInputs) {}

    public default void zeroYaw() {}
}
