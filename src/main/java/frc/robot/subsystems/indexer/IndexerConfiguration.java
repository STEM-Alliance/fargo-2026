package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class IndexerConfiguration {
    public static final TalonFXConfiguration kIndexerMotorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLowerTime(Seconds.zero())
        );
}
