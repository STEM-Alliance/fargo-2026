package frc.robot.subsystems.shooter.kicker.io;

import static frc.robot.subsystems.shooter.ShooterConfiguration.KickerConfiguration.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public class KickerIOReal implements KickerIO {
    private final TalonFX m_kickerMotor;
    private final VoltageOut m_kickerMotorSetpoint;

    private final StatusSignal<Angle> m_kickerMotorPosition;
    private final StatusSignal<AngularVelocity> m_kickerMotorVelocity;
    private final StatusSignal<Voltage> m_kickerMotorAppliedVoltage;
    private final StatusSignal<Current> m_kickerMotorStatorCurrent;

    public KickerIOReal(int kickerMotorID) {
        m_kickerMotor = new TalonFX(kickerMotorID);

        m_kickerMotorSetpoint = new VoltageOut(0.0)
            .withUpdateFreqHz(0.0)
            .withEnableFOC(true);

        m_kickerMotorPosition = m_kickerMotor.getPosition(false);
        m_kickerMotorVelocity = m_kickerMotor.getVelocity(false);
        m_kickerMotorAppliedVoltage = m_kickerMotor.getMotorVoltage(false);
        m_kickerMotorStatorCurrent = m_kickerMotor.getStatorCurrent(false);

        m_kickerMotor.getConfigurator().apply(kKickerMotorConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_kickerMotorPosition,
            m_kickerMotorVelocity,
            m_kickerMotorAppliedVoltage,
            m_kickerMotorStatorCurrent
        );

        m_kickerMotor.optimizeBusUtilization(0.0);
    }

    @Override
    public final void updateInputs(KickerInputs loggableInputs) {
        loggableInputs.isKickerMotorConnected = BaseStatusSignal.refreshAll(
            m_kickerMotorPosition,
            m_kickerMotorVelocity,
            m_kickerMotorAppliedVoltage,
            m_kickerMotorStatorCurrent
        ).isOK();

        loggableInputs.kickerMotorPosition = m_kickerMotorPosition.getValue();
        loggableInputs.kickerMotorVelocity = m_kickerMotorVelocity.getValue();
        loggableInputs.kickerMotorAppliedVoltage = m_kickerMotorAppliedVoltage.getValue();
        loggableInputs.kickerMotorStatorCurrent = m_kickerMotorStatorCurrent.getValue();
    }

    @Override
    public final void setKickerMotorVoltage(Voltage voltage) {
        m_kickerMotor.setControl(m_kickerMotorSetpoint.withOutput(voltage));
    }
}
