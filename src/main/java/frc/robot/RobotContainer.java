// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOReal;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.ShooterUtils;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandXboxController m_programmerController = new CommandXboxController(5);

    private final Field2d m_field = new Field2d();
    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final DrivetrainSubsystem m_drivetrain;
    private final VisionSubsystem m_vision;

    public RobotContainer() {
        switch (RobotConstants.BEHAVIOR) {
            case REAL -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOReal(13),
                    new SwerveModuleIOReal(kModuleConfigurations[0]),
                    new SwerveModuleIOReal(kModuleConfigurations[1]),
                    new SwerveModuleIOReal(kModuleConfigurations[2]),
                    new SwerveModuleIOReal(kModuleConfigurations[3])
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOReal("FrontCAM", new Transform3d(
                        new Translation3d(0.0254, 0.0, 0.279),
                        new Rotation3d(0.0, 0.0, 0.0)
                    )),

                    new VisionIOReal("BackCAM", new Transform3d(
                        new Translation3d(-0.0254, 0.0, 0.279),
                        new Rotation3d(0.0, 0.0, Math.PI)
                    ))
                );
            }

            case SIMULATED -> {
                var drivetrainSimulation = new SwerveDriveSimulation(kSimulationConfig, kSimulationStartingPose);

                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOSim(13, drivetrainSimulation.getGyroSimulation()),
                    new SwerveModuleIOSim(kModuleConfigurations[0], drivetrainSimulation.getModules()[0]),
                    new SwerveModuleIOSim(kModuleConfigurations[1], drivetrainSimulation.getModules()[1]),
                    new SwerveModuleIOSim(kModuleConfigurations[2], drivetrainSimulation.getModules()[2]),
                    new SwerveModuleIOSim(kModuleConfigurations[3], drivetrainSimulation.getModules()[3]),
                    drivetrainSimulation
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOSim(
                        "PerfectCAM",
                        new Transform3d(),
                        new SimCameraProperties(),
                        m_drivetrain::getSimulationPose
                    )
                );

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            }

            default -> throw new UnsupportedOperationException();
        }

        CommandScheduler.getInstance().registerSubsystem(m_drivetrain, m_vision);

        configurePathPlanner();
        configureBindings();
        configureAutoChooser();

        SmartDashboard.putData("Field2D", m_field);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        configureAlerts();
    }

    /** Sets up subsystem and controller command bindings. */
    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));
        m_driverController.rightTrigger().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));

        m_driverController.x().onTrue(Commands.runOnce(() -> {m_drivetrain.zeroYaw();})); // Temporary.
        // m_driverController.rightTrigger().whileTrue(new AutoAimCommand(m_driverController, m_drivetrain));

        // m_programmerController.a().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
        // m_programmerController.b().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
        // m_programmerController.rightTrigger().whileTrue(new ControllerDriveCommand(m_programmerController, m_drivetrain));
//  SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
//                 m_drivetrain.getSimulationPose().getTranslation(),
//                 new Translation2d(),
//                 ChassisSpeeds.fromRobotRelativeSpeeds(m_drivetrain.getChassisSpeeds(), m_drivetrain.getSimulationPose().getRotation()),
//                 m_drivetrain.getSimulationPose().getRotation(),
//                 Meters.of(0.762),
//                 MetersPerSecond.of(8.0),
//                 ShooterUtils.getQuadraticAngles(
//                     Meters.of(
//                         ShooterUtils.getLeadedTranslation(
//                             m_drivetrain.getEstimatedPose(),
//                             FieldUtils.getAllianceHub(),
//                             MetersPerSecond.of(8.0),
//                             m_drivetrain.getChassisSpeeds()
//                         ).getNorm()
//                     ),
//                     Meters.of(1.58),
//                     MetersPerSecond.of(8.0)
//                     ).getSecond()
        if (RobotConstants.isSimulated()) {
            m_driverController.leftTrigger().whileTrue(Commands.repeatingSequence(
                Commands.runOnce(() -> {
                    System.out.println("Shot");
                    Pose2d pose = m_drivetrain.getEstimatedPose();
                    boolean passing = !FieldUtils.inFriendlyAllianceZone(pose);
                    Translation2d target = ShooterUtils.getShooterTarget(pose, null, null);

                    SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                        m_drivetrain.getSimulationPose().getTranslation(),
                        Translation2d.kZero,
                        ChassisSpeeds.fromRobotRelativeSpeeds(m_drivetrain.getChassisSpeeds(), m_drivetrain.getSimulationPose().getRotation()),
                        passing ? target.minus(pose.getTranslation()).getAngle() : ShooterUtils.getLeadedTranslation(pose, target, MetersPerSecond.of(8), m_drivetrain.getChassisSpeeds()).getAngle(),
                        Meters.of(0.762),
                        MetersPerSecond.of(8.0),
                        passing ? Degrees.of(22.5) :
                        ShooterUtils.getQuadraticAngles(
                            Meters.of(
                                ShooterUtils.getLeadedTranslation(
                                    pose,
                                    target,
                                    MetersPerSecond.of(8.0),
                                    m_drivetrain.getChassisSpeeds()
                                ).getNorm()
                            ),
                            Meters.of(1.58),
                            MetersPerSecond.of(8.0)
                        ).getSecond()
                    ));
                }),

                Commands.waitSeconds(1.0 / 3.0)
            ));
        }
    }

    /** Registers PathPlanner's {@link NamedCommands} and sets up field telemetry. */
    private final void configurePathPlanner() {
        CommandScheduler.getInstance().schedule(
            Commands.run(
                () -> m_field.setRobotPose(m_drivetrain.getEstimatedPose())
            ).ignoringDisable(true)
        );

        PathPlannerLogging.setLogTargetPoseCallback(
            pose -> m_field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(
            poses -> m_field.getObject("path").setPoses(poses)
        );
    }

    /** Creates the AutoChooser's selectable autonomous modes. */
    private final void configureAutoChooser() {
        m_autoChooser.addDefaultOption("None", Commands.none());

        m_autoChooser.addOption("Test", new PathPlannerAuto("Test"));
    }

    /** Sets defaults for alerts and sets up their triggers. */
    private final void configureAlerts() {
        m_autoChooserAlert.set(true);
        m_autoChooser.getSendableChooser().onChange(
            key ->  m_autoChooserAlert.set(key.equals("None"))
        );
    }

    public final Command getAutonomousCommand() {
        return m_autoChooser.get();
    }
}
