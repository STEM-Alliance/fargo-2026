package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.shooter.flywheel.io.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelinputsAutoLogged;

public class Flywheel implements Subsystem {
    private final FlywheelIO m_flywheelIO;
    private final FlywheelinputsAutoLogged m_flywheelInputs;

    public Flywheel(FlywheelIO flywheelIO) {
        m_flywheelIO = flywheelIO;
        m_flywheelInputs = new FlywheelinputsAutoLogged();
    }

    public final void periodic() {
        m_flywheelIO.updateInputs(m_flywheelInputs);
        Logger.processInputs("ShooterSubsystem/Flywheel", m_flywheelInputs);
    }

    public final void setMotorVelocities(AngularVelocity velocity) {
        if (velocity.abs(RotationsPerSecond) > 1e-6) {
            m_flywheelIO.setMotorVelocities(velocity);
        } else {
            stopMotors();
        }
    }

    public final void setMotorVoltages(Voltage voltage) {
        m_flywheelIO.setMotorVoltages(voltage);
    }

    public final void stopMotors() {
        m_flywheelIO.setMotorVoltages(Volts.zero());
    }
}
