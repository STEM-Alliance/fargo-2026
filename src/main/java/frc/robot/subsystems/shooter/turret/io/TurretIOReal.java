package frc.robot.subsystems.shooter.turret.io;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.subsystems.shooter.turret.TurretConfig;
import frc.robot.utils.FoyerDevice;

public class TurretIOReal implements TurretIO {
    protected final TalonFX m_turretMotor;
    protected final TalonFX m_hoodMotor;
    protected final FoyerDevice m_foyer;
    protected final DigitalInput m_hoodHomingSwitch;

    private final MotionMagicVoltage m_turretMotorSetpoint;
    private final MotionMagicVoltage m_hoodMotorSetpoint;

    private final StatusSignal<Angle> m_turretMotorPosition;
    private final StatusSignal<Voltage> m_turretMotorAppliedVoltage;
    private final StatusSignal<Current> m_turretMotorStatorCurrent;

    private final StatusSignal<Angle> m_hoodMotorPosition;
    private final StatusSignal<Voltage> m_hoodMotorAppliedVoltage;
    private final StatusSignal<Current> m_hoodMotorStatorCurrent;

    public TurretIOReal(TurretConfig configuration) {
        m_turretMotor = new TalonFX(configuration.turretMotorID());
        m_hoodMotor = new TalonFX(configuration.hoodMotorID());
        m_foyer = new FoyerDevice(configuration.turretEncoderID());
        m_hoodHomingSwitch = new DigitalInput(configuration.hoodHomingSwitchID());

        m_turretMotor.getConfigurator().apply(kTurretMotorConfiguration);
        m_hoodMotor.getConfigurator().apply(kHoodMotorConfiguration);

        m_turretMotorSetpoint = new MotionMagicVoltage(0.0)
            .withUpdateFreqHz(100.0)
            .withEnableFOC(true);

        m_hoodMotorSetpoint = new MotionMagicVoltage(0.0)
            .withUpdateFreqHz(100.0)
            .withEnableFOC(true);

        m_turretMotorPosition = m_turretMotor.getPosition(false);
        m_turretMotorAppliedVoltage = m_turretMotor.getMotorVoltage(false);
        m_turretMotorStatorCurrent = m_turretMotor.getStatorCurrent(false);

        m_hoodMotorPosition = m_hoodMotor.getPosition(false);
        m_hoodMotorAppliedVoltage = m_hoodMotor.getMotorVoltage(false);
        m_hoodMotorStatorCurrent = m_hoodMotor.getStatorCurrent(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_turretMotorAppliedVoltage,
            m_turretMotorStatorCurrent,
            m_hoodMotorAppliedVoltage,
            m_hoodMotorStatorCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_turretMotorPosition,
            m_hoodMotorPosition
        );

        ParentDevice.optimizeBusUtilizationForAll(m_turretMotor, m_hoodMotor);
    }

    @Override
    public void updateInputs(TurretInputs loggableInputs) {
        loggableInputs.isTurretEncoderConnected = BaseStatusSignal.refreshAll(
            m_turretMotorPosition,
            m_turretMotorAppliedVoltage,
            m_turretMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.turretMotorPosition = m_turretMotorPosition.getValue();
        loggableInputs.turretMotorAppliedVoltage = m_turretMotorAppliedVoltage.getValue();
        loggableInputs.turretMotorStatorCurrent = m_turretMotorStatorCurrent.getValue();

        if (m_foyer.isEncoderConnected()) {
            loggableInputs.isTurretEncoderConnected = true;
            loggableInputs.turretEncoderPosition = Degrees.of(
                m_foyer.getEncoderStatus().enc1AbsDeg()
            );
        } else {
            loggableInputs.isTurretEncoderConnected = false;
        }

        loggableInputs.isHoodMotorConnected = BaseStatusSignal.refreshAll(
            m_hoodMotorPosition,
            m_hoodMotorAppliedVoltage,
            m_hoodMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.hoodMotorPosition = m_hoodMotorPosition.getValue();
        loggableInputs.hoodMotorAppliedVoltage = m_hoodMotorAppliedVoltage.getValue();
        loggableInputs.hoodMotorStatorCurrent = m_hoodMotorStatorCurrent.getValue();

        loggableInputs.isHoodHomingSwitchPressed = m_hoodHomingSwitch.get();
    }

    @Override
    public void setTurretAzimuth(Angle azimuth) {
        m_turretMotor.setControl(m_turretMotorSetpoint.withPosition(
            azimuth.times(kTurretMotorRatio)
        ));
    }

    @Override
    public void setHoodAngle(Angle angle) {
        m_hoodMotor.setControl(m_hoodMotorSetpoint.withPosition(
            angle.times(kHoodMotorRatio)
        ));
    }

    @Override
    public void setTurretMotorPosition(Angle position) {
        m_turretMotor.setPosition(position);
    }

    @Override
    public void setHoodMotorPosition(Angle position) {
        m_hoodMotor.setPosition(position);
    }

    @Override
    public void setHoodMotorVoltage(Voltage voltage) {
        m_hoodMotor.setVoltage(voltage.in(Volts));
    }
}
