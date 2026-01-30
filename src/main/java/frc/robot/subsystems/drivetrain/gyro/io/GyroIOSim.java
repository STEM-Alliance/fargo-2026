package frc.robot.subsystems.drivetrain.gyro.io;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.sim.Pigeon2SimState;

public class GyroIOSim extends GyroIOReal {
    private final GyroSimulation m_gyroSimulation;
    private final Pigeon2SimState m_pigeonSimState;

    public GyroIOSim(int gyroID, GyroSimulation gyroSimulation) {
        super(gyroID);

        m_gyroSimulation = gyroSimulation;
        m_pigeonSimState = m_pigeon.getSimState();
    }

    @Override
    public void updateInputs(GyroInputs loggableInputs) {
        m_pigeonSimState.setRawYaw(m_gyroSimulation.getGyroReading().getDegrees());
        m_pigeonSimState.setAngularVelocityZ(m_gyroSimulation.getMeasuredAngularVelocity());
        m_pigeonSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        super.updateInputs(loggableInputs);
    }
}
