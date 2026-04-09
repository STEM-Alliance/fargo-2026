package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.shooter.flywheel.FlywheelHardware;
import frc.robot.subsystems.shooter.turret.TurretHardware;

public final class ShooterConfiguration {
    public static class KickerConfiguration {
        public static final TalonFXConfiguration kKickerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80.0))
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLowerTime(Seconds.zero())
            );
    }

    public static class TurretConfiguration {
        public static final double kTurretRingRatio = 3.5;
        public static final double kTurretMotorRatio = 25.0;
        public static final double kHoodDegToMotorRad = -3.88 / (84.0 - 64.0);
        public static final Translation2d kTurretOffset = new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-3.0));

        public static final Angle kTurretZeroOffset = Radians.of(-Math.PI * kTurretMotorRatio * kTurretRingRatio);
        public static final Angle kHoodMotorZero = Radians.of(84.0 * kHoodDegToMotorRad);

        public static final TurretHardware kTurretHardware = new TurretHardware(
            21, 5, 24
        );

        public static final TalonFXSConfiguration kTurretMotorConfiguration = new TalonFXSConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(1.500).withKI(0.000).withKD(0.000)
                .withKS(0.000).withKV(0.100).withKA(0.000) // TODO: Add KS, tune KV, up accel.
                .withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(kTurretMotorRatio * kTurretRingRatio * 2))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(750.0))
            ).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Radians.of(120.0))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Radians.of(-860.0))
                .withReverseSoftLimitEnable(true)
            ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                .withGainSchedErrorThreshold(Rotations.of(0.1))
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLowerTime(Seconds.zero())
            ).withCommutation(new CommutationConfigs()
                .withMotorArrangement(MotorArrangementValue.Minion_JST)
            );

        public static final TalonFXSConfiguration kHoodMotorConfiguration = new TalonFXSConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(7.500).withKI(0.000).withKD(0.000)
                .withKS(0.000).withKV(0.000).withKA(0.000)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(7.5)) // drivetrain 1/2 rot/s. 87.5/2 to match
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(500.0)) // 800
            ).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Radians.of(-13.35))
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(kHoodMotorZero)
                .withReverseSoftLimitEnable(true)
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLowerTime(Seconds.zero())
            ).withCommutation(new CommutationConfigs()
                .withMotorArrangement(MotorArrangementValue.Minion_JST)
            );
    }

    public static class FlywheelConfiguration {
        public static final FlywheelHardware kFlywheelHardware = new FlywheelHardware(22, 23);

        public static final TalonFXConfiguration kLeftFlywheelMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                // Tune kS for stationary motion.
                // Set flywheel to setpoint, tune kV and then kP until no oscillation.
                // Then lower and adjust kP down again for oscillation.
                .withKP(9.000).withKI(0.000).withKD(0.000)
                .withKS(1.900).withKV(0.042).withKA(0.000) // 0.035 and 10 for p
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120.0))
                .withSupplyCurrentLimit(Amps.of(60.0))
                .withSupplyCurrentLowerLimit(Amps.of(40.0))
                .withSupplyCurrentLowerTime(Seconds.of(2.5))
            ).withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(Volts.of(10.0))
                .withPeakReverseVoltage(Volts.of(-10.0))
            ).withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            );

        public static final TalonFXConfiguration kRightFlywheelMotorConfiguration = kLeftFlywheelMotorConfiguration
            .clone().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
            );
    }
}
