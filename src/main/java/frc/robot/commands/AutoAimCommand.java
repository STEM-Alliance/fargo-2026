package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.kMaxLinearSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.ShooterUtils;

public final class AutoAimCommand extends Command {
    private final CommandXboxController m_controller;
    private final DrivetrainSubsystem m_drivetrain;

    private final ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds();
    private final ProfiledPIDController m_omegaController = new ProfiledPIDController(
        5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0 * Math.PI, 2.0 * Math.PI / 0.5)
    );

    public AutoAimCommand(
        CommandXboxController controller,
        DrivetrainSubsystem drivetrain
    ) {
        m_controller = controller;
        m_drivetrain = drivetrain;

        m_omegaController.enableContinuousInput(-Math.PI, Math.PI);

        setName("AutoAimCommand");
        addRequirements(m_drivetrain);
    }

    @Override
    public final void initialize() {
        m_omegaController.reset(m_drivetrain.getEstimatedPose().getRotation().getRadians());
    }

    @Override
    public final void execute() {
        double leftX = m_controller.getLeftX();
        double leftY = m_controller.getLeftY();

        // For the left stick (translation), filtering is done to the input magnitude instead of each input indivually.
        // This means that the deadband and exponential scaling are applied circularly instead of on a square. Most jo-
        // ysticks aren't perfectly circular, so some diagonal range is lost. We could calibrate and use a superellipse
        // to map each quadrant of either stick to a perfect cirle, but that is unnecessary for what we need.
        double leftMagnitude = MathUtil.applyDeadband(Math.hypot(leftX, leftY), 0.1);

        if (leftMagnitude == 0.0) {
            leftX = 0.0;
            leftY = 0.0;
        } else {
            // We divide by the unclamped magnitude to scale the resulting magnitude to at most 1.0.
            double scaledLeftMagnitude = getScaledInput(leftMagnitude);
            leftX = leftX / leftMagnitude * scaledLeftMagnitude;
            leftY = leftY / leftMagnitude * scaledLeftMagnitude;
        }

        Translation2d leadVector = ShooterUtils.getLeadedTranslation(
            m_drivetrain.getEstimatedPose(),
            FieldUtils.getAllianceHub(),
            MetersPerSecond.of(8.0),
            m_drivetrain.getChassisSpeeds(),
            0
        );

        // In the WPILib coordinate system, +X is forward and +Y is left (relative to
        // the blue driver station), so the controller X and Y are flipped and inverted.
        // This assumes that the drive function is also using blue origin coordinates.
        m_desiredSpeeds.vxMetersPerSecond = -leftY * kMaxLinearSpeed.in(MetersPerSecond);
        m_desiredSpeeds.vyMetersPerSecond = -leftX * kMaxLinearSpeed.in(MetersPerSecond);
        m_desiredSpeeds.omegaRadiansPerSecond = -m_omegaController.calculate(
            m_drivetrain.getEstimatedPose().getRotation().getRadians(),
            leadVector.getAngle().getRadians()
        );

        m_drivetrain.drive(m_desiredSpeeds, true, FieldUtils.getAlliance() == Alliance.Blue);
    }

    /**
     * Gets the input, clamped to [-1.0, 1.0], scaled via the scaling function below.
     * <p> Currently, the scaling function is a semi-circle multiplied by the sign of the input.
     * 
     * @param input The input to scale.
    */
    private final double getScaledInput(double input) {
        input = MathUtil.clamp(input, -1.0, 1.0);

        // This is a personal preference and should be changed for different drivers.
        // The only requirement is that the function is odd: f(x) shouldn't equal f(-x).
        return Math.copySign(1.0 - Math.sqrt(1.0 - Math.pow(input, 2.0)), input);
    }
}
