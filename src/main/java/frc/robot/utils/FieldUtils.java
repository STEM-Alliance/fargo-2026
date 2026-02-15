package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldUtils {
    private FieldUtils() {}

    private static final AprilTagFieldLayout m_aprilTagLayout;
    private static final Translation2d m_blueAllianceHub;
    private static final Translation2d m_redAllianceHub;

    public static final Distance kHubWidth;
    public static final Distance kFieldLength;
    public static final Distance kFieldWidth;
    public static final Distance kFieldWidthCenter;

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

        kHubWidth = Inches.of(47.0);
        kFieldWidth = Meters.of(m_aprilTagLayout.getFieldWidth());
        kFieldLength = Meters.of(m_aprilTagLayout.getFieldLength());
        kFieldWidthCenter = kFieldWidth.div(2.0);
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

    // switch this to using a raycast method with an "expansion" on the square of the hubs.
    public static boolean inlineWithHubs(Pose2d pose, double tolerance) {
        return (pose.getY() >= m_aprilTagLayout.getTagPose(5).get().getY() - tolerance) &&
               (pose.getY() <= m_aprilTagLayout.getTagPose(2).get().getY() + tolerance);
    }
}
