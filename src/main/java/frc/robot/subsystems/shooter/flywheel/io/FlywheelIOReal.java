// package frc.robot.subsystems.shooter.flywheel.io;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;

// import frc.robot.subsystems.shooter.flywheel.FlywheelConfig;

// public class FlywheelIOReal {
//     private final TalonFX m_leftFlywheelMotor;
//     private final Follower m_rightFlywheelMotor;

//     private final StatusSignal<Angle> m_leftFlywheelMotorPosition;
//     private final StatusSignal<AngularVelocity> m_leftFlywheelMotorVelocity;
//     private final StatusSignal<Voltage> m_leftFlywheelMotorAppliedVoltage;
//     private final StatusSignal<Current> m_leftFlywheelMotorStatorCurrent;

//     private final StatusSignal<Angle> m_rightFlywheelMotorPosition;
//     private final StatusSignal<AngularVelocity> m_rightFlywheelMotorPosiiton;
//     private final StatusSignal<Voltage> m_rightFlywheelMotorAppliedVoltage;
//     private final StatusSignal<Current> m_rightFlywheelMotorStatorCurrent;

//     public FlywheelIOReal(FlywheelConfig configuration) {
//         m_leftFlywheelMotor = new TalonFX(configuration.leftFlywheelMotorID());
//         m_rightFlywheelMotor = new Follower(
//             configuration.leftFlywheelMotorID(),
//             MotorAlignmentValue.Opposed
//         );
//     }
// }

// //     private final TalonFXS m_intakeMotor;
// //     private final TalonFXS m_agitatorMotor;

// //     private final StatusSignal<Angle> m_intakeMotorPosition;
// //     private final StatusSignal<AngularVelocity> m_intakeMotorVelocity;
// //     private final StatusSignal<Voltage> m_intakeMotorAppliedVoltage;
// //     private final StatusSignal<Current> m_intakeMotorStatorCurrent;

// //     private final StatusSignal<Angle> m_agitatorMotorPosition;
// //     private final StatusSignal<AngularVelocity> m_agitatorMotorVelocity;
// //     private final StatusSignal<Voltage> m_agitatorMotorAppliedVoltage;
// //     private final StatusSignal<Current> m_agitatorMotorStatorCurrent;

// //     public IntakeIOReal(IntakeConfig configuration) {
// //         m_compressor = new Compressor(PneumaticsModuleType.REVPH);

// //         m_leftSolenoid = new DoubleSolenoid(
// //             PneumaticsModuleType.REVPH,
// //             configuration.leftSolenoidForwardID(),
// //             configuration.leftSolenoidReverseID()
// //         );

// //         m_rightSolenoid = new DoubleSolenoid(
// //             PneumaticsModuleType.REVPH,
// //             configuration.rightSolenoidForwardID(),
// //             configuration.rightSolenoidReverseID()
// //         );

// //         m_intakeMotor = new TalonFXS(configuration.intakeMotorID());
// //         m_agitatorMotor = new TalonFXS(configuration.agitatorMotorID());

// //         m_compressor.enableDigital();
// //         m_intakeMotor.getConfigurator().apply(kIntakeMotorConfiguration);
// //         m_agitatorMotor.getConfigurator().apply(kAgitatorMotorConfiguration);

// //         m_intakeMotorPosition = m_intakeMotor.getPosition(false);
// //         m_intakeMotorVelocity = m_intakeMotor.getVelocity(false);
// //         m_intakeMotorAppliedVoltage = m_intakeMotor.getMotorVoltage(false);
// //         m_intakeMotorStatorCurrent = m_intakeMotor.getStatorCurrent(false);

// //         m_agitatorMotorPosition = m_agitatorMotor.getPosition(false);
// //         m_agitatorMotorVelocity = m_agitatorMotor.getVelocity(false);
// //         m_agitatorMotorAppliedVoltage = m_agitatorMotor.getMotorVoltage(false);
// //         m_agitatorMotorStatorCurrent = m_agitatorMotor.getStatorCurrent(false);

// //         BaseStatusSignal.setUpdateFrequencyForAll(
// //             50.0,
// //             m_intakeMotorPosition,
// //             m_intakeMotorVelocity,
// //             m_intakeMotorAppliedVoltage,
// //             m_intakeMotorStatorCurrent,
// //             m_agitatorMotorPosition,
// //             m_agitatorMotorVelocity,
// //             m_agitatorMotorAppliedVoltage,
// //             m_agitatorMotorStatorCurrent
// //         );

// //         ParentDevice.optimizeBusUtilizationForAll(m_intakeMotor, m_agitatorMotor);
// //     }

// //     @Override
// //     public final void updateInputs(IntakeInputs loggableInputs) {
// //         loggableInputs.isIntakeExtended = m_leftSolenoid.get() == Value.kForward;

// //         loggableInputs.isIntakeMotorConnected = BaseStatusSignal.refreshAll(
// //             m_intakeMotorPosition,
// //             m_intakeMotorVelocity,
// //             m_intakeMotorAppliedVoltage,
// //             m_intakeMotorStatorCurrent
// //         ) == StatusCode.OK;

// //         loggableInputs.intakeMotorPosition = m_intakeMotorPosition.getValue();
// //         loggableInputs.intakeMotorVelocity = m_intakeMotorVelocity.getValue();
// //         loggableInputs.intakeMotorAppliedVoltage = m_intakeMotorAppliedVoltage.getValue();
// //         loggableInputs.intakeMotorStatorCurrent = m_intakeMotorStatorCurrent.getValue();

// //         loggableInputs.isAgitatorMotorConnected = BaseStatusSignal.refreshAll(
// //             m_agitatorMotorPosition,
// //             m_agitatorMotorVelocity,
// //             m_agitatorMotorAppliedVoltage,
// //             m_agitatorMotorStatorCurrent
// //         ) == StatusCode.OK;

// //         loggableInputs.agitatorMotorPosition = m_agitatorMotorPosition.getValue();
// //         loggableInputs.agitatorMotorVelocity = m_agitatorMotorVelocity.getValue();
// //         loggableInputs.agitatorMotorAppliedVoltage = m_agitatorMotorAppliedVoltage.getValue();
// //         loggableInputs.agitatorMotorStatorCurrent = m_agitatorMotorStatorCurrent.getValue();
// //     }

// //     @Override
// //     public final void setIntakeExtended(boolean extended) {
// //         m_leftSolenoid.set(extended ? Value.kForward : Value.kReverse);
// //         m_rightSolenoid.set(extended ? Value.kForward : Value.kReverse);
// //     }

// //     @Override
// //     public final void setIntakeMotorVoltage(Voltage voltage) {
// //         m_intakeMotor.setVoltage(voltage.in(Volts));
// //     }

// //     @Override
// //     public final void setAgitatorMotorVoltage(Voltage voltage) {
// //         m_agitatorMotor.setVoltage(voltage.in(Volts));
// //     }
// // }
