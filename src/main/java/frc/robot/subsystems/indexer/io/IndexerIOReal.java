package frc.robot.subsystems.indexer.io;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConfiguration.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX m_indexerMotor;

    private final StatusSignal<Angle> m_indexerMotorPosition;
    private final StatusSignal<AngularVelocity> m_indexerMotorVelocity;
    private final StatusSignal<Voltage> m_indexerMotorAppliedVoltage;
    private final StatusSignal<Current> m_indexerMotorStatorCurrent;

    public IndexerIOReal(int indexerMotorID) {
        m_indexerMotor = new TalonFX(indexerMotorID);

        m_indexerMotor.getConfigurator().apply(kIndexerMotorConfiguration);

        m_indexerMotorPosition = m_indexerMotor.getPosition(false);
        m_indexerMotorVelocity = m_indexerMotor.getVelocity(false);
        m_indexerMotorAppliedVoltage = m_indexerMotor.getMotorVoltage(false);
        m_indexerMotorStatorCurrent = m_indexerMotor.getStatorCurrent(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_indexerMotorPosition,
            m_indexerMotorVelocity,
            m_indexerMotorAppliedVoltage,
            m_indexerMotorStatorCurrent
        );

        m_indexerMotor.optimizeBusUtilization();
    }

    @Override
    public final void updateInputs(IndexerInputs loggableInputs) {
        loggableInputs.isIndexerMotorConnected = BaseStatusSignal.refreshAll(
            m_indexerMotorPosition,
            m_indexerMotorVelocity,
            m_indexerMotorAppliedVoltage,
            m_indexerMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.indexerMotorPosition = m_indexerMotorPosition.getValue();
        loggableInputs.indexerMotorVelocity = m_indexerMotorVelocity.getValue();
        loggableInputs.indexerMotorAppliedVoltage = m_indexerMotorAppliedVoltage.getValue();
        loggableInputs.indexerMotorStatorCurrent = m_indexerMotorStatorCurrent.getValue();
    }

    @Override
    public final void setIndexerMotorVoltage(Voltage voltage) {
        m_indexerMotor.setVoltage(voltage.in(Volts));
    }
}
