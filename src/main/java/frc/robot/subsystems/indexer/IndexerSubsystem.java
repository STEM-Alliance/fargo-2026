package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.indexer.io.IndexerIO;
import frc.robot.subsystems.indexer.io.IndexerInputsAutoLogged;

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

    public final void start() {
        m_indexerIO.setIndexerMotorVoltage(Volts.of(-3.5));
    }

    public final void stop() {
        m_indexerIO.setIndexerMotorVoltage(Volts.zero());
    }
}
