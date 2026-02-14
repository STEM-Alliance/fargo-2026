package frc.robot.subsystems.shooter.turret.io;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.*;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.subsystems.shooter.turret.TurretConfig;

public final class TurretIOSim extends TurretIOReal {
    private final TalonFXSimState m_turretMotorSimState;
    private final TalonFXSimState m_hoodMotorSimState;

    private final DCMotorSim m_turretMotorSimulation = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getMinion(1),
            0.015,
            kTurretMotorRatio
        ),

        DCMotor.getMinion(1)
    );

    private final DCMotorSim m_hoodMotorSimulation = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNeo550(1),
            0.005,
            kHoodMotorRatio
        ),

        DCMotor.getNeo550(1)
    );

    public TurretIOSim(TurretConfig configuration) {
        super(configuration);

        m_turretMotorSimState = m_turretMotor.getSimState();
        m_hoodMotorSimState = m_hoodMotor.getSimState();

        SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_turretMotorSimulation.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_hoodMotorSimulation.getCurrentDrawAmps()));
    }

    @Override
    public final void updateInputs(TurretInputs loggableInputs) {
        m_turretMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
        m_turretMotorSimulation.setInputVoltage(m_turretMotorSimState.getMotorVoltage());
        m_turretMotorSimulation.update(RobotConstants.kLoopPeriod.in(Seconds));
        m_turretMotorSimState.setRawRotorPosition(m_turretMotorSimulation.getAngularPosition().times(kTurretMotorRatio));

        m_hoodMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
        m_hoodMotorSimulation.setInputVoltage(m_hoodMotorSimState.getMotorVoltage());
        m_hoodMotorSimulation.update(RobotConstants.kLoopPeriod.in(Seconds));
        m_hoodMotorSimState.setRawRotorPosition(m_hoodMotorSimulation.getAngularPosition().times(kHoodMotorRatio));

        super.updateInputs(loggableInputs);

        if (loggableInputs.hoodMotorPosition.in(Degrees) / kHoodMotorRatio >= 90.0) {
            loggableInputs.isHoodHomingSwitchPressed = true;
        } else {
            loggableInputs.isHoodHomingSwitchPressed = false;
        }
    }
}
