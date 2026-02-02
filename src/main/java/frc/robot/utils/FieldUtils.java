package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldUtils {
    private FieldUtils() {}

    private static final AprilTagFieldLayout m_aprilTagLayout;
    private static final Translation2d m_blueAllianceHub;
    private static final Translation2d m_redAllianceHub;
    private static final Translation2d[] m_passingTargets;

    static {
        // Uncomment the following to use calibrated field layouts.
        // AprilTagFieldLayout aprilTagLayout;

        // try {
        //     aprilTagLayout = new AprilTagFieldLayout(
        //         Filesystem.getDeployDirectory().toPath().resolve("2026xxxx.json")
        //     );
        // } catch (IOException e) {
        //     DriverStation.reportWarning("Failed to load custom AprilTag layout.", true);
        //     aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        // }

        //m_aprilTagLayout = aprilTagLayout;
        m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        m_blueAllianceHub = new Translation2d(
            m_aprilTagLayout.getTagPose(18).get().getX(),
            m_aprilTagLayout.getTagPose(20).get().getY()
        );

        m_redAllianceHub = new Translation2d(
            m_aprilTagLayout.getTagPose(2).get().getX(),
            m_aprilTagLayout.getTagPose(4).get().getY()
        );

        double fieldWidth = m_aprilTagLayout.getFieldWidth();

        double blueMidline =
            (m_aprilTagLayout.getTagPose(26).get().getX() +
             m_aprilTagLayout.getTagPose(31).get().getX()) / 2.0;

        double redMidline =
            (m_aprilTagLayout.getTagPose(10).get().getX() +
             m_aprilTagLayout.getTagPose(15).get().getX()) / 2.0;

        m_passingTargets = new Translation2d[] {
            // Quadrants are blue-relative with (0, 0) in the bottom left.
            new Translation2d(redMidline, fieldWidth * 4.0 / 5.0), // Q1
            new Translation2d(blueMidline, fieldWidth * 4.0 / 5.0), // Q2
            new Translation2d(blueMidline, fieldWidth * 1.0 / 5.0), // Q3
            new Translation2d(redMidline, fieldWidth * 1.0 / 5.0), // Q4
            new Translation2d(blueMidline, fieldWidth / 2.0), // Blue Center
            new Translation2d(redMidline, fieldWidth / 2.0) // Red Center
        };
    }

    /**
     * Gets the current AprilTag layout being used.
     * 
     * @return The static {@link AprilTagFieldLayout} instance.
    */
    public static AprilTagFieldLayout getAprilTagLayout() {
        return m_aprilTagLayout;
    }

    /**
     * Gets the robot's alliance as reported by the driver station.
     * 
     * @return The reported alliance, defaulting to {@code Blue} if invalid.
    */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /**
     * Returns a boolean for whether the robot is on the blue alliance or not.
     * <p> Shorthand equivalent to {@code FieldUtils.getAlliance() == Alliance.Blue}.
     * 
     * @return {@code true} if the robot is on the blue alliance and {@code false} otherwise.
    */
    public static boolean isBlueAlliance() {
        return getAlliance() == Alliance.Blue;
    }

    /**
     * Gets the {@link Translation2d} of the alliance's corresponding hub.
     * <p> The hub translation is determined via the translations of its AprilTags.
     * 
     * @return The field-relative {@link Translation2d} of the alliance's hub.
    */
    public static Translation2d getAllianceHub() {
        return isBlueAlliance() ? m_blueAllianceHub : m_redAllianceHub;
    }

    public static Translation2d getPassingTarget(Pose2d pose, boolean passInner) {
        if (passInner) {
            return m_passingTargets[isBlueAlliance() ? 4 : 5];
        } else {
            if (pose.getY() > m_aprilTagLayout.getFieldWidth() / 2.0) {
                return m_passingTargets[isBlueAlliance() ? 1 : 0];
            } else {
                return m_passingTargets[isBlueAlliance() ? 2 : 3];
            }
        }
    }

    public static boolean inBlueAllianceZone(Pose2d pose) {
        return pose.getX() <= m_aprilTagLayout.getTagPose(21).get().getX();
    }

    public static boolean inRedAllianceZone(Pose2d pose) {
        return pose.getX() >= m_aprilTagLayout.getTagPose(2).get().getX();
    }

    public static boolean inFriendlyAllianceZone(Pose2d pose) {
        return (isBlueAlliance() && inBlueAllianceZone(pose)) ||
               (!isBlueAlliance() && inRedAllianceZone(pose));
    }

    public static boolean inNeutralZone(Pose2d pose) {
        return !inBlueAllianceZone(pose) && !inRedAllianceZone(pose);
    }

    public static boolean shotIntersectsHub(Pose2d pose, double tolerance) {
        Translation2d hub = getAllianceHub();
        Translation2d shotTarget = m_passingTargets[isBlueAlliance() ? 4 : 5];
        Translation2d shotPath = shotTarget.minus(pose.getTranslation());

        double t = hub.minus(pose.getTranslation()).dot(shotPath) / shotPath.getSquaredNorm();
        t = Math.max(Math.min(t, 1.0), 0.0);

        Translation2d nearest = pose.getTranslation().plus(shotPath.times(t)).minus(hub);

        return (Math.abs(nearest.getX()) < tolerance) && (Math.abs(nearest.getY()) < tolerance);
    }

    // switch this to using a raycast method with an "expansion" on the square of the hubs.
    public static boolean inlineWithHubs(Pose2d pose, double tolerance) {
        return (pose.getY() >= m_aprilTagLayout.getTagPose(5).get().getY() - tolerance) &&
               (pose.getY() <= m_aprilTagLayout.getTagPose(2).get().getY() + tolerance);
    }
}
