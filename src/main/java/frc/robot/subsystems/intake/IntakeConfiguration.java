package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class IntakeConfiguration {
    public static final IntakeHardware kIntakeHardware = new IntakeHardware(
        30, 1, 0
    );

    public static final TalonFXSConfiguration kIntakeMotorConfiguration = new TalonFXSConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLowerTime(Seconds.zero())
        ).withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        );
}
