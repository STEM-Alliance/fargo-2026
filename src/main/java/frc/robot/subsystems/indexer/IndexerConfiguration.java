package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class IndexerConfiguration {
    public static final IndexerHardware kIndexerHardware = new IndexerHardware(34);

    public static final TalonFXConfiguration kIndexerMotorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLowerTime(Seconds.zero())
        ).withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        );
}
