package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;

public final class RobotVisualizer {
    private static  Transform3d m_turretTransform = new Transform3d();

    public static final void updateComponents() {
        // For now, components cannot be "linked" in AdvantageScope; if you have
        // multiple DOF, transforms should be summed in the function call below.
        Logger.recordOutput("RobotComponents", m_turretTransform);
    }

    public static final void setTurretTransform(Transform3d turretTransform) {
        m_turretTransform = turretTransform;
    }
}
