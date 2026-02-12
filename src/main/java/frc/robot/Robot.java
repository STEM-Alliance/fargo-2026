// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotConstants.*;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Robot extends LoggedRobot {
    private final RobotContainer m_robotContainer;

    private Command m_autonomousCommand = Commands.none();

    public Robot() {
        super(kLoopPeriod.in(Seconds));

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes committed.";
            case 1 -> "Uncommitted changes.";
            default -> "Unknown.";
        });

        switch (RobotConstants.getBehavior()) {
            case COMPETITION -> {
                Logger.addDataReceiver(new WPILOGWriter());
            }

            case DEVELOPMENT -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }

            case SIMULATION -> {
                Logger.addDataReceiver(new NT4Publisher());
            }

            case LOG_REPLAY -> {
                String readPath = LogFileUtil.findReplayLog();
                String writePath = LogFileUtil.addPathSuffix(readPath, "_replayed");

                Logger.setReplaySource(new WPILOGReader(readPath));
                Logger.addDataReceiver(new WPILOGWriter(writePath));
                setUseTiming(false);
            }

            default -> throw new UnsupportedOperationException();
        }

        Logger.start();

        // We don't need CTRE's logging; AdvantageKit logs all useful information.
        SignalLogger.enableAutoLogging(false);

        // If we aren't in a match, disconnected joysticks aren't important to us.
        DriverStation.silenceJoystickConnectionWarning(true);

        // Loop overruns that only span the duration of a a few loops are trivial.
        CommandScheduler.getInstance().setPeriod(kLoopWatchdogPeriod.in(Seconds));

        // This allows for some warmup time before setting the main thread priority.
        // If our main thread is too slow, this can cause issues with other threads.
        // Assuming low enough CPU usage, this will ultimately reduce loop overruns.
        // Doesn't this comment block look so neatly aligned? It's rather exquisite!
        CommandScheduler.getInstance().schedule(Commands.sequence(
            Commands.waitSeconds(10.0),
            Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 99))
        ).ignoringDisable(true));

        m_robotContainer = new RobotContainer();
    }

    @Override
    public final void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
    }

    @Override
    public final void disabledInit() {}

    @Override
    public final void disabledPeriodic() {}

    @Override
    public final void disabledExit() {}

    @Override
    public final void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public final void autonomousPeriodic() {}

    @Override
    public final void autonomousExit() {}

    @Override
    public final void teleopInit() {
        CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    @Override
    public final void teleopPeriodic() {}

    @Override
    public final void teleopExit() {}

    @Override
    public final void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public final void testPeriodic() {}

    @Override
    public final void testExit() {}

    @Override
    public final void simulationInit() {
        if (!RobotConstants.isLogReplay()) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    @Override
    public final void simulationPeriodic() {
        if (!RobotConstants.isLogReplay()) {
            SimulatedArena.getInstance().simulationPeriodic();

            Logger.recordOutput(
                "SimulatedArena/FuelPoses",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")
            );
        }
    }
}
