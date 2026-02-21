package frc.robot.subsystems.shooter.flywheel.io;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.subsystems.shooter.flywheel.FlywheelConfig;

public class FlywheelIOReal implements FlywheelIO {
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    private final MotionMagicVelocityVoltage m_leftMotorSetpoint;
    private final Follower m_rightMotorSetpoint;

    private final StatusSignal<Angle> m_leftMotorPosition;
    private final StatusSignal<AngularVelocity> m_leftMotorVelocity;
    private final StatusSignal<Voltage> m_leftMotorAppliedVoltage;
    private final StatusSignal<Current> m_leftMotorStatorCurrent;

    private final StatusSignal<Angle> m_rightMotorPosition;
    private final StatusSignal<AngularVelocity> m_rightMotorVelocity;
    private final StatusSignal<Voltage> m_rightMotorAppliedVoltage;
    private final StatusSignal<Current> m_rightMotorStatorCurrent;

    public FlywheelIOReal(FlywheelConfig configuration) {
        m_leftMotor = new TalonFX(configuration.leftMotorID());
        m_rightMotor = new TalonFX(configuration.rightMotorID());

        m_leftMotor.getConfigurator().apply(kFlywheelMotorsConfiguration);
        m_rightMotor.getConfigurator().apply(kFlywheelMotorsConfiguration);

        m_leftMotorSetpoint = new MotionMagicVelocityVoltage(0.0)
            .withUpdateFreqHz(100.0)
            .withEnableFOC(true);

        m_rightMotorSetpoint = new Follower(
            configuration.leftMotorID(),
            MotorAlignmentValue.Opposed
        ).withUpdateFreqHz(100.0);

        m_leftMotorPosition = m_leftMotor.getPosition(false);
        m_leftMotorVelocity = m_leftMotor.getVelocity(false);
        m_leftMotorAppliedVoltage = m_leftMotor.getMotorVoltage(false);
        m_leftMotorStatorCurrent = m_leftMotor.getStatorCurrent(false);

        m_rightMotorPosition = m_rightMotor.getPosition(false);
        m_rightMotorVelocity = m_rightMotor.getVelocity(false);
        m_rightMotorAppliedVoltage = m_rightMotor.getMotorVoltage(false);
        m_rightMotorStatorCurrent = m_rightMotor.getStatorCurrent(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_leftMotorPosition,
            m_leftMotorStatorCurrent,
            m_rightMotorPosition,
            m_rightMotorStatorCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_leftMotorVelocity,
            m_leftMotorAppliedVoltage,
            m_rightMotorVelocity,
            m_rightMotorStatorCurrent
        );

        ParentDevice.optimizeBusUtilizationForAll(m_leftMotor, m_rightMotor);

        m_rightMotor.setControl(m_rightMotorSetpoint);
    }

    @Override
    public final void updateInputs(Flywheelinputs loggableInputs) {
        loggableInputs.isLeftMotorConnected = BaseStatusSignal.refreshAll(
            m_leftMotorPosition,
            m_leftMotorVelocity,
            m_leftMotorAppliedVoltage,
            m_leftMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.leftMotorPosition = m_leftMotorPosition.getValue();
        loggableInputs.leftMotorVelocity = m_leftMotorVelocity.getValue();
        loggableInputs.leftMotorAppliedVoltage = m_leftMotorAppliedVoltage.getValue();
        loggableInputs.leftMotorStatorCurrent = m_leftMotorStatorCurrent.getValue();

        loggableInputs.isRightMotorConnected = BaseStatusSignal.refreshAll(
            m_rightMotorPosition,
            m_rightMotorVelocity,
            m_rightMotorAppliedVoltage,
            m_rightMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.rightMotorPosition = m_rightMotorPosition.getValue();
        loggableInputs.rightMotorVelocity = m_rightMotorVelocity.getValue();
        loggableInputs.rightMotorAppliedVoltage = m_rightMotorAppliedVoltage.getValue();
        loggableInputs.rightMotorStatorCurrent = m_rightMotorStatorCurrent.getValue();
    }

    @Override
    public final void setMotorVelocities(AngularVelocity velocity) {
        // The follower will automatically update with the left motor.
        m_leftMotor.setControl(m_leftMotorSetpoint.withVelocity(velocity));
    }

    @Override
    public final void setMotorVoltages(Voltage voltage) {
        // The follower will automatically update with the left motor.
        m_leftMotor.setVoltage(voltage.in(Volts));
    }
}
