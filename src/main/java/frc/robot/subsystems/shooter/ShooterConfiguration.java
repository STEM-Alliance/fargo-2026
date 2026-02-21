package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.turret.TurretConfig;

public final class ShooterConfiguration {
    public static final double kTurretMotorRatio = 3.5 * 25.0;
    public static final double kHoodMotorRatio = 36.0;

    // TODO: Record zero
    public static final double kTurretEncoderZero = Units.radiansToRotations(5.811);

    // TODO: Record zero
    public static final TurretConfig kTurretConfiguration = new TurretConfig(
        21, 6, 22, 1
    );

    // TODO: Tune, CTRE software limits + setpoint unwrapping.
    public static final TalonFXSConfiguration kTurretMotorConfiguration = new TalonFXSConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(10.0).withKI(0.000).withKD(0.000) // P=25
            .withKS(0.000).withKV(0.000).withKA(0.000)
            .withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(300.0)) // 500 each
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(300.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        ).withCommutation(new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.Minion_JST)
        ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withGainSchedErrorThreshold(Rotations.of(25.0))
        ).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Rotations.of(190.0 * kTurretMotorRatio))
            .withReverseSoftLimitThreshold(Rotations.of(-190.0 * kTurretMotorRatio))
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
        );

    // TODO: Tune, CTRE & setpoint software limits.
    public static final TalonFXConfiguration kHoodMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(0.625).withKI(0.000).withKD(0.000)
            .withKS(0.000).withKV(0.000).withKA(0.000)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(9999.0))
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(9999.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(20.0))
            .withSupplyCurrentLimit(Amps.of(20.0))
        );

    public static final TalonFXConfiguration kFlywheelMotorsConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(1.000).withKI(0.000).withKD(0.000)
            .withKS(0.000).withKV(0.000).withKA(0.000)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(25.0))
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(9999.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        );
}
