package frc.robot.subsystems.shooter.turret.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

/**
 * The generic turret IO interface (with AdvantageKit support).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface TurretIO {
    @AutoLog
    public static abstract class TurretInputs {
        public boolean isTurretMotorConnected = false;
        public Angle turretMotorPosition = Radians.of(0.0);
        public Voltage turretMotorAppliedVoltage = Volts.of(0.0);
        public Current turretMotorStatorCurrent = Amps.of(0.0);

        public boolean isTurretEncoderConnected = false;
        public Angle turretEncoderPosition = Radians.of(0.0);

        public boolean isHoodMotorConnected = false;
        public Angle hoodMotorPosition = Radians.of(0.0);
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
     * Sets the turret motor's onboard position setpoint from a given turret angle.
     * 
     * @param azimuth The desired angle of the turret relative to the encoder's zero.
    */
    public default void setTurretAzimuth(Angle azimuth) {}

    /**
     * Sets the hood motor's onboard position setpoint from a given launch angle.
     * 
     * @param angle The desired launch angle of the hood relative to the horizontal.
    */
    public default void setHoodAngle(Angle angle) {}

    /**
     * Resets the turret motor's encoder position to the supplied position.
     * 
     * @param position The position to reset the motor's encoder to, relative to itself.
    */
    public default void setTurretMotorPosition(Angle position) {}

    /**
     * Resets the hood motor's encoder position to the supplied position.
     * 
     * @param position The position to reset the motor's encoder to, relative to itself.
    */
    public default void setHoodMotorPosition(Angle position) {}

    /**
     * Directly sets the applied voltage to the hood motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the hood motor.
    */
    public default void setHoodMotorVoltage(Voltage voltage) {}
}
