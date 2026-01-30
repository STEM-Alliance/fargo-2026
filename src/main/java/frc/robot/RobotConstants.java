package frc.robot;

public final class RobotConstants {
    public static final RobotBehavior BEHAVIOR =
        Robot.isReal() ? RobotBehavior.REAL : RobotBehavior.SIMULATED;

    public static enum RobotBehavior {
        REAL,
        SIMULATED
    }
}
