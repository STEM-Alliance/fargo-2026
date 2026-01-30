package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldUtils {
    private static final AprilTagFieldLayout m_aprilTagLayout;
    private static final Translation2d m_blueAllianceHub;
    private static final Translation2d m_redAllianceHub;

    static {
        m_aprilTagLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2026RebuiltWelded
        );

        m_blueAllianceHub = new Translation2d(
            m_aprilTagLayout.getTagPose(18).get().getX(),
            m_aprilTagLayout.getTagPose(20).get().getY()
        );

        m_redAllianceHub = new Translation2d(
            m_aprilTagLayout.getTagPose(2).get().getX(),
            m_aprilTagLayout.getTagPose(4).get().getY()
        );
    }

    public static final AprilTagFieldLayout getAprilTagLayout() {
        return m_aprilTagLayout;
    }

    public static final Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static final Translation2d getAllianceHub() {
        if (getAlliance() == Alliance.Blue) {
            return m_blueAllianceHub;
        } else {
            return m_redAllianceHub;
        }
    }
}
