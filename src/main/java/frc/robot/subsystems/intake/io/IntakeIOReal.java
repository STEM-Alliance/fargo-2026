package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.subsystems.intake.IntakeConfiguration.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.robot.subsystems.intake.IntakeHardware;

public class IntakeIOReal implements IntakeIO {
    private final Compressor m_compressor;
    private final TalonFXS m_intakeMotor;
    private final DoubleSolenoid m_intakeSolenoid;
    private final VoltageOut m_intakeMotorSetpoint;

    private final StatusSignal<Angle> m_intakeMotorPosition;
    private final StatusSignal<AngularVelocity> m_intakeMotorVelocity;
    private final StatusSignal<Voltage> m_intakeMotorAppliedVoltage;
    private final StatusSignal<Current> m_intakeMotorStatorCurrent;

    public IntakeIOReal(IntakeHardware hardware) {
        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        m_intakeMotor = new TalonFXS(hardware.intakeMotorID());

        m_intakeSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            hardware.intakeSolenoidForwardID(),
            hardware.intakeSolenoidReverseID()
        );

        m_intakeMotorSetpoint = new VoltageOut(0.0)
            .withUpdateFreqHz(50.0) // TODO: Test with 0hz updating.
            .withEnableFOC(true);

        m_intakeMotorPosition = m_intakeMotor.getPosition(false);
        m_intakeMotorVelocity = m_intakeMotor.getVelocity(false);
        m_intakeMotorAppliedVoltage = m_intakeMotor.getMotorVoltage(false);
        m_intakeMotorStatorCurrent = m_intakeMotor.getStatorCurrent(false);

        m_compressor.enableDigital();
        m_intakeMotor.getConfigurator().apply(kIntakeMotorConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_intakeMotorPosition,
            m_intakeMotorVelocity,
            m_intakeMotorAppliedVoltage,
            m_intakeMotorStatorCurrent
        );

        m_intakeMotor.optimizeBusUtilization(0.0);
    }

    @Override
    public void updateInputs(IntakeInputs loggableInputs) {
        loggableInputs.compressorVoltage = Volts.of(m_compressor.getAnalogVoltage());
        loggableInputs.compressorCurrent = Amps.of(m_compressor.getCurrent());

        loggableInputs.isIntakeSolenoidForward = m_intakeSolenoid.get() == kForward;

        loggableInputs.isIntakeMotorConnected = BaseStatusSignal.refreshAll(
            m_intakeMotorPosition,
            m_intakeMotorVelocity,
            m_intakeMotorAppliedVoltage,
            m_intakeMotorStatorCurrent
        ).isOK();

        loggableInputs.intakeMotorPosition = m_intakeMotorPosition.getValue();
        loggableInputs.intakeMotorVelocity = m_intakeMotorVelocity.getValue();
        loggableInputs.intakeMotorAppliedVoltage = m_intakeMotorAppliedVoltage.getValue();
        loggableInputs.intakeMotorStatorCurrent = m_intakeMotorStatorCurrent.getValue();
    }

    @Override
    public void setIntakeExtended(boolean extended) {
        m_intakeSolenoid.set(extended ? kForward : kReverse);
    }

    @Override
    public void setIntakeMotorVoltage(Voltage voltage) {
        m_intakeMotor.setControl(m_intakeMotorSetpoint.withOutput(voltage));
    }
}
