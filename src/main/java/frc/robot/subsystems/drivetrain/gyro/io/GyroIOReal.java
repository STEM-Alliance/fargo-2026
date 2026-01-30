package frc.robot.subsystems.drivetrain.gyro.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class GyroIOReal implements GyroIO {
    protected final Pigeon2 m_pigeon;

    private final StatusSignal<Voltage> m_supplyVoltage;
    private final StatusSignal<Angle> m_pitch;
    private final StatusSignal<AngularVelocity> m_pitchRate;
    private final StatusSignal<Angle> m_roll;
    private final StatusSignal<AngularVelocity> m_rollRate;
    private final StatusSignal<Angle> m_yaw;
    private final StatusSignal<AngularVelocity> m_yawRate;

    public GyroIOReal(int gyroID) {
        m_pigeon = new Pigeon2(gyroID);

        m_supplyVoltage = m_pigeon.getSupplyVoltage(false);
        m_pitch = m_pigeon.getPitch(false);
        m_pitchRate = m_pigeon.getAngularVelocityXWorld(false);
        m_roll = m_pigeon.getRoll(false);
        m_rollRate = m_pigeon.getAngularVelocityYWorld(false);
        m_yaw = m_pigeon.getYaw(false);
        m_yawRate = m_pigeon.getAngularVelocityZWorld(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_supplyVoltage,
            m_pitch,
            m_pitchRate,
            m_roll,
            m_rollRate,
            m_yaw,
            m_yawRate
        );

        m_pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs loggableInputs) {
        loggableInputs.isConnected = BaseStatusSignal.refreshAll(
            m_supplyVoltage,
            m_pitch,
            m_pitchRate,
            m_roll,
            m_rollRate,
            m_yaw,
            m_yawRate
        ) == StatusCode.OK;

        loggableInputs.supplyVoltage = m_supplyVoltage.getValue();
        loggableInputs.pitch = m_pitch.getValue();
        loggableInputs.pitchRate = m_pitchRate.getValue();
        loggableInputs.roll = m_roll.getValue();
        loggableInputs.rollRate = m_rollRate.getValue();
        loggableInputs.yaw = m_yaw.getValue();
        loggableInputs.yawRate = m_yawRate.getValue();
    }

    @Override
    public final void zeroYaw() {
        m_pigeon.setYaw(0.0);
    }
}
