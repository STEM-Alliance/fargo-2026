package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.subsystems.shooter.turret.TurretConfig;

public final class ShooterConfiguration {
    public static final double kTurretMotorRatio = 3.5;
    public static final double kHoodMotorRatio = 36.0;

    public static final TurretConfig kTurretConfiguration = new TurretConfig(13, 14, 15, 16);

    public static final TalonFXConfiguration kTurretMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(20.00).withKI(0.000).withKD(1.000)
            .withKS(0.000).withKV(0.000).withKA(0.000)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(250.0))
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(500.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(20.0))
            .withSupplyCurrentLimit(Amps.of(20.0))
        );

    public static final TalonFXConfiguration kHoodMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(0.625).withKI(0.000).withKD(0.000)
            .withKS(0.000).withKV(0.000).withKA(0.000)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(250.0))
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(500.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(20.0))
            .withSupplyCurrentLimit(Amps.of(20.0))
        );
}
