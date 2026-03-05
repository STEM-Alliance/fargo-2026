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
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.shooter.flywheel.FlywheelHardware;
import frc.robot.subsystems.shooter.turret.TurretHardware;

public final class ShooterConfiguration {
    public static class KickerConfiguration {}

    public static class TurretConfiguration {
        public static final double kTurretRingRatio = 3.5;
        public static final double kTurretMotorRatio = 25.0;
        public static final double kHoodDegToMotorRot = 11.838;
        public static final Translation2d kTurretOffset = new Translation2d(0.0, 0.1651);

        public static final Angle kTurretEncoderZero = Rotations.of(0.8273);
        public static final Angle kHoodMotorZero = Rotations.of(62.0 * kHoodDegToMotorRot);

        public static final TurretHardware kTurretHardware = new TurretHardware(
            21, 6, 24
        );

        public static final TalonFXSConfiguration kTurretMotorConfiguration = new TalonFXSConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(5.000).withKI(0.000).withKD(0.000)
                .withKS(0.100).withKV(0.110).withKA(0.000)
                .withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(500.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(350.0))
            ).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Rotations.of(190.0 * kTurretRingRatio * kTurretMotorRatio))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Rotations.of(-190.0 * kTurretRingRatio * kTurretMotorRatio))
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
                .withKP(1.000).withKI(0.000).withKD(0.000)
                .withKS(0.000).withKV(0.010).withKA(0.000)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(250.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(800.0))
            ).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Rotations.of(61.0 * kHoodDegToMotorRot))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Rotations.of(22.5 * kHoodDegToMotorRot))
                .withReverseSoftLimitEnable(true)
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(15.0))
                .withSupplyCurrentLimit(Amps.of(15.0))
                .withSupplyCurrentLowerTime(Seconds.zero())
            ).withCommutation(new CommutationConfigs()
                .withMotorArrangement(MotorArrangementValue.NEO550_JST)
            );
    }

    public static class FlywheelConfiguration {
        public static final FlywheelHardware kFlywheelHardware = new FlywheelHardware(22, 23);

        public static final TalonFXConfiguration kFlywheelMotorsConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.125).withKI(0.000).withKD(0.000)
                .withKS(0.000).withKV(0.115).withKA(0.000)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(500.0))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1000.0))
            ).withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withSupplyCurrentLowerTime(Seconds.zero())
            ).withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            );
    }
}
