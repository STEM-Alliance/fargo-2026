package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.subsystems.intake.IntakeConfiguration.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.robot.subsystems.intake.IntakeConfig;

public class IntakeIOReal implements IntakeIO {
    protected final Compressor m_compressor;
    protected final DoubleSolenoid m_leftSolenoid;
    protected final DoubleSolenoid m_rightSolenoid;

    protected final TalonFXS m_intakeMotor;
    protected final TalonFXS m_agitatorMotor;

    private final StatusSignal<Angle> m_intakeMotorPosition;
    private final StatusSignal<AngularVelocity> m_intakeMotorVelocity;
    private final StatusSignal<Voltage> m_intakeMotorAppliedVoltage;
    private final StatusSignal<Current> m_intakeMotorStatorCurrent;

    private final StatusSignal<Angle> m_agitatorMotorPosition;
    private final StatusSignal<AngularVelocity> m_agitatorMotorVelocity;
    private final StatusSignal<Voltage> m_agitatorMotorAppliedVoltage;
    private final StatusSignal<Current> m_agitatorMotorStatorCurrent;

    public IntakeIOReal(IntakeConfig configuration) {
        m_compressor = new Compressor(PneumaticsModuleType.REVPH);

        m_leftSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            configuration.leftSolenoidForwardID(),
            configuration.leftSolenoidReverseID()
        );

        m_rightSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            configuration.rightSolenoidForwardID(),
            configuration.rightSolenoidReverseID()
        );

        m_intakeMotor = new TalonFXS(configuration.intakeMotorID());
        m_agitatorMotor = new TalonFXS(configuration.agitatorMotorID());

        m_compressor.enableDigital();
        m_intakeMotor.getConfigurator().apply(kIntakeMotorConfiguration);
        m_agitatorMotor.getConfigurator().apply(kAgitatorMotorConfiguration);

        m_intakeMotorPosition = m_intakeMotor.getPosition(false);
        m_intakeMotorVelocity = m_intakeMotor.getVelocity(false);
        m_intakeMotorAppliedVoltage = m_intakeMotor.getMotorVoltage(false);
        m_intakeMotorStatorCurrent = m_intakeMotor.getStatorCurrent(false);

        m_agitatorMotorPosition = m_agitatorMotor.getPosition(false);
        m_agitatorMotorVelocity = m_agitatorMotor.getVelocity(false);
        m_agitatorMotorAppliedVoltage = m_agitatorMotor.getMotorVoltage(false);
        m_agitatorMotorStatorCurrent = m_agitatorMotor.getStatorCurrent(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_intakeMotorPosition,
            m_intakeMotorVelocity,
            m_intakeMotorAppliedVoltage,
            m_intakeMotorStatorCurrent,

            m_agitatorMotorPosition,
            m_agitatorMotorVelocity,
            m_agitatorMotorAppliedVoltage,
            m_agitatorMotorStatorCurrent
        );

        ParentDevice.optimizeBusUtilizationForAll(m_intakeMotor, m_agitatorMotor);
    }

    @Override
    public void updateInputs(IntakeInputs loggableInputs) {
        loggableInputs.compressorPressure = m_compressor.getPressure();
        loggableInputs.compressorVoltage = Volts.of(m_compressor.getAnalogVoltage());
        loggableInputs.compressorCurrent = Amps.of(m_compressor.getCurrent());

        loggableInputs.isIntakeExtended = m_leftSolenoid.get() == kForward;

        loggableInputs.isIntakeMotorConnected = BaseStatusSignal.refreshAll(
            m_intakeMotorPosition,
            m_intakeMotorVelocity,
            m_intakeMotorAppliedVoltage,
            m_intakeMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.intakeMotorPosition = m_intakeMotorPosition.getValue();
        loggableInputs.intakeMotorVelocity = m_intakeMotorVelocity.getValue();
        loggableInputs.intakeMotorAppliedVoltage = m_intakeMotorAppliedVoltage.getValue();
        loggableInputs.intakeMotorStatorCurrent = m_intakeMotorStatorCurrent.getValue();

        loggableInputs.isAgitatorMotorConnected = BaseStatusSignal.refreshAll(
            m_agitatorMotorPosition,
            m_agitatorMotorVelocity,
            m_agitatorMotorAppliedVoltage,
            m_agitatorMotorStatorCurrent
        ) == StatusCode.OK;

        loggableInputs.agitatorMotorPosition = m_agitatorMotorPosition.getValue();
        loggableInputs.agitatorMotorVelocity = m_agitatorMotorVelocity.getValue();
        loggableInputs.agitatorMotorAppliedVoltage = m_agitatorMotorAppliedVoltage.getValue();
        loggableInputs.agitatorMotorStatorCurrent = m_agitatorMotorStatorCurrent.getValue();
    }

    @Override
    public void setIntakeExtended(boolean extended) {
        m_leftSolenoid.set(extended ? kForward: kReverse);
        m_rightSolenoid.set(extended ? kForward : kReverse);
    }

    @Override
    public void setIntakeMotorVoltage(Voltage voltage) {
        m_intakeMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setAgitatorMotorVoltage(Voltage voltage) {
        m_agitatorMotor.setVoltage(voltage.in(Volts));
    }
}
