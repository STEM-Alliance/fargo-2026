package frc.robot.subsystems.drivetrain.swervemodule.io;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleConfig;

public class SwerveModuleIOSim extends SwerveModuleIOReal {
    public SwerveModuleIOSim(SwerveModuleConfig configuration, SwerveModuleSimulation simulation) {
        super(configuration);

        simulation.useDriveMotorController(new DriveMotorTalonFXSim(m_driveMotor));
        simulation.useSteerMotorController(new AzimuthMotorTalonFXSim(m_azimuthMotor, m_azimuthEncoder));
    }

    private static final class DriveMotorTalonFXSim implements SimulatedMotorController {
        private final TalonFXSimState m_driveMotorSimState;

        public DriveMotorTalonFXSim(TalonFX driveMotor) {
            m_driveMotorSimState = driveMotor.getSimState();
        }

        @Override
        public final Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity
        ) {
            m_driveMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            m_driveMotorSimState.setRawRotorPosition(encoderAngle);
            m_driveMotorSimState.setRotorVelocity(encoderVelocity);

            return m_driveMotorSimState.getMotorVoltageMeasure();
        }
    }

    private static final class AzimuthMotorTalonFXSim implements SimulatedMotorController {
        private final TalonFXSimState m_azimuthMotorSimState;
        private final CANcoderSimState m_azimuthEncoderSimState;

        public AzimuthMotorTalonFXSim(TalonFX azimuthMotor, CANcoder azimuthEncoder) {
            m_azimuthMotorSimState = azimuthMotor.getSimState();
            m_azimuthEncoderSimState = azimuthEncoder.getSimState();
        }

        @Override
        public final Voltage updateControlSignal(
            Angle mechanismAngle,
            AngularVelocity mechanismVelocity,
            Angle encoderAngle,
            AngularVelocity encoderVelocity
        ) {
            m_azimuthEncoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            m_azimuthEncoderSimState.setRawPosition(mechanismAngle);
            m_azimuthEncoderSimState.setVelocity(mechanismVelocity);

            m_azimuthMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            m_azimuthMotorSimState.setRawRotorPosition(encoderAngle);
            m_azimuthMotorSimState.setRotorVelocity(encoderVelocity);

            return m_azimuthMotorSimState.getMotorVoltageMeasure();
        }
    }
}
