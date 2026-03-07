package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

public final class IntakeConfiguration {
    public static final IntakeHardware kIntakeHardware = new IntakeHardware(
        30, 0, 1
    );

    public static final TalonFXSConfiguration kIntakeMotorConfiguration = new TalonFXSConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLowerTime(Seconds.zero())
        );

    public static final TalonFXSConfiguration kAgitatorMotorConfiguration = new TalonFXSConfiguration();
}
