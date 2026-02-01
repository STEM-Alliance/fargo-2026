package frc.robot.subsystems.shooter.flywheel.io;

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
 * The generic flywheel IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface FlywheelIO {
    @AutoLog
    public static abstract class Flywheelinputs {
        public boolean isLeftMotorConnected = false;
        public Angle leftMotorPosition = Radians.of(0.0);
        public AngularVelocity leftMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration leftMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage leftMotorAppliedVoltage = Volts.of(0.0);
        public Current leftMotorStatorCurrent = Amps.of(0.0);

        public boolean isRightMotorConnected = false;
        public Angle rightMotorPosition = Radians.of(0.0);
        public AngularVelocity rightMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration rightMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage rightMotorAppliedVoltage = Volts.of(0.0);
        public Current rightMotorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(Flywheelinputs loggableInputs) {}

    /**
     * Sets both of the flywheel motors' onboard velocity setpoints.
     * 
     * @param velocity The desired angular velocity of the flywheel motors.
    */
    public default void setMotorVelocities(AngularVelocity velocity) {}

    /**
     * Directly sets the applied voltage to the flywheel motors.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the flywheel motors.
    */
    public default void setMotorVoltages(Voltage voltage) {}
}
