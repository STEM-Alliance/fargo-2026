package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.indexer.io.IndexerIO;
import frc.robot.subsystems.indexer.io.IndexerInputsAutoLogged;
import frc.robot.subsystems.shooter.ShotCalculator;

public final class IndexerSubsystem implements Subsystem {
    private final IndexerIO m_indexerIO;
    private final IndexerInputsAutoLogged m_indexerInputs;

    public IndexerSubsystem(IndexerIO indexerIO) {
        m_indexerIO = indexerIO;
        m_indexerInputs = new IndexerInputsAutoLogged();
    }

    @Override
    public final void periodic() {
        m_indexerIO.updateInputs(m_indexerInputs);
        Logger.processInputs("IndexerSubsystem", m_indexerInputs);
    }

    public final void setRunning(boolean running, boolean reverse, boolean slow) {
        // TODO: Make indexder speed distance dependant.
        var angle = ShotCalculator.getLaunchAngle().in(Degrees);
        slow = (Double.isFinite(angle)) && (angle >= 75.0);
        var voltage = slow ? 1.5 : 3.0;
        m_indexerIO.setIndexerMotorVoltage(
            running ? Volts.of(reverse ? voltage : -voltage) : Volts.zero()
        );
    }
}
