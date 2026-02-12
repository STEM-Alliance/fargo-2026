package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public final class RobotConstants {
    // There are some small tweaks that may want to be made between development
    // and during a competition, like disabling some NetworkTables information.
    public static final RobotBehavior kRealBehavior = RobotBehavior.DEVELOPMENT;
    public static final RobotBehavior kSimulationBehavior = RobotBehavior.SIMULATION;
    public static final Time kLoopPeriod = Seconds.of(0.02);
    public static final Time kLoopWatchdogPeriod = Seconds.of(0.08);

    public enum RobotBehavior {
        COMPETITION,
        DEVELOPMENT,
        SIMULATION,
        LOG_REPLAY
    }

    public static boolean isReal() {
        return Robot.isReal();
    }

    public static boolean isCompetition() {
        return isReal() && kRealBehavior == RobotBehavior.COMPETITION;
    }

    public static boolean isSimulated() {
        return Robot.isSimulation();
    }

    public static boolean isLogReplay() {
        return isSimulated() && kSimulationBehavior == RobotBehavior.LOG_REPLAY;
    }

    public static RobotBehavior getBehavior() {
        return isReal() ? kRealBehavior : kSimulationBehavior;
    }
}
