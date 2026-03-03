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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.shooter.turret.TurretConfig;

public final class ShooterConfiguration {
    public static final double kTurretMotorRatio = 25.0;
    public static final double kTurretRingRatio = 3.5;
    public static final double kHoodMotorRatio = 60.0; // 60 on stack,
    public static final Translation2d kTurretOffset = new Translation2d(0.0, Units.inchesToMeters(6.5));

    public static final Angle kTurretEncoderZero = Radians.of(5.198);

    // TODO: Record zero
    public static final TurretConfig kTurretConfiguration = new TurretConfig(
        21, 6, 24, 1
    );

    // TODO: Tune, CTRE software limits + setpoint unwrapping.
    public static final TalonFXSConfiguration kTurretMotorConfiguration = new TalonFXSConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(5.000).withKI(0.000).withKD(0.000) // P=25
            .withKS(0.100).withKV(0.11).withKA(0.000) // kV is volts per rps
            .withGainSchedBehavior(GainSchedBehaviorValue.ZeroOutput)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0.0)) // 350
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.0)) // 500
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        ).withCommutation(new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.Minion_JST)
        ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withGainSchedErrorThreshold(Rotations.of(0.5))
        ).withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Rotations.of(190.0 * kTurretMotorRatio * kTurretRingRatio))
            .withReverseSoftLimitThreshold(Rotations.of(-190.0 * kTurretMotorRatio * kTurretRingRatio))
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
        ).withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        );

    // TODO: Tune, CTRE & setpoint software limits.
    public static final TalonFXSConfiguration kHoodMotorConfiguration = new TalonFXSConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(1.000).withKI(0.000).withKD(0.000)
            .withKS(0.000).withKV(0.010).withKA(0.000)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(800.0)) // 1000
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(250.0)) // 250
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(15.0))
            .withSupplyCurrentLimit(Amps.of(15.0))
            .withSupplyCurrentLowerLimit(Amps.of(10.0))
            .withSupplyCurrentLowerTime(Seconds.of(2.5))
        ).withCommutation(new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.NEO550_JST)
        );

    public static final TalonFXConfiguration kFlywheelMotorsConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(0.125).withKI(0.000).withKD(0.000)
            .withKS(0.000).withKV(0.115).withKA(0.000)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1000.0))
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(500.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        );
}
