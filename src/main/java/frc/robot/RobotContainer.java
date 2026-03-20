// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;
import static frc.robot.subsystems.indexer.IndexerConfiguration.*;
import static frc.robot.subsystems.intake.IntakeConfiguration.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.FlywheelConfiguration.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.TurretConfiguration.*;

import java.util.Objects;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.ShooterControlCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOReal;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.io.IndexerIO;
import frc.robot.subsystems.indexer.io.IndexerIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOReal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIOReal;
import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.kicker.io.KickerIOReal;
import frc.robot.subsystems.shooter.turret.io.TurretIO;
import frc.robot.subsystems.shooter.turret.io.TurretIOReal;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.ShiftTimeUtil;
import frc.robot.utils.RobotVisualizer;
import frc.robot.utils.ShooterUtils;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_programmerController = new CommandXboxController(5);

    private final Field2d m_field = new Field2d();
    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final VisionSubsystem m_vision;
    private final DrivetrainSubsystem m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;

    public RobotContainer() {
        switch (RobotConstants.getBehavior()) {
            case COMPETITION, DEVELOPMENT -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOReal(0),
                    new SwerveModuleIOReal(kModuleConfigurations[0]),
                    new SwerveModuleIOReal(kModuleConfigurations[1]),
                    new SwerveModuleIOReal(kModuleConfigurations[2]),
                    new SwerveModuleIOReal(kModuleConfigurations[3])
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),

                    new VisionIOReal("BackLeftCamera", new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12.1875), Units.inchesToMeters(9.3125), Units.inchesToMeters(12.5)),
                        new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(180.0 - 45.0))
                    )),

                    new VisionIOReal("BackRightCamera", new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12.1875), Units.inchesToMeters(-9.3125), Units.inchesToMeters(12.5)),
                        new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(180.0 + 45.0))
                    ))
                );

                m_intake = new IntakeSubsystem(new IntakeIOReal(kIntakeHardware));
                m_indexer = new IndexerSubsystem(new IndexerIOReal(kIndexerHardware));

                m_shooter = new ShooterSubsystem(
                    new KickerIOReal(37),
                    new TurretIOReal(kTurretHardware),
                    new FlywheelIOReal(kFlywheelHardware)
                );
            }

            case SIMULATION -> {
                // TODO: implement basic simulation of all subsystems.
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

                m_intake = new IntakeSubsystem(
                    new IntakeIO() {}
                );

                m_indexer = new IndexerSubsystem(
                    new IndexerIO() {}
                );

                m_shooter = new ShooterSubsystem(
                    new KickerIO() {},
                    new TurretIO() {},
                    new FlywheelIO() {}
                );

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            }

            case LOG_REPLAY -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {}
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIO() {},
                    new VisionIO() {}
                );

                m_intake = new IntakeSubsystem(
                    new IntakeIO() {}
                );

                m_indexer = new IndexerSubsystem(
                    new IndexerIO() {}
                );

                m_shooter = new ShooterSubsystem(
                    new KickerIO() {},
                    new TurretIO() {},
                    new FlywheelIO() {}
                );
            }

            default -> throw new UnsupportedOperationException();
        }

        // TODO: Look into pre/post scheduler updates like 6328.
        CommandScheduler.getInstance().registerSubsystem(
            m_vision,
            m_drivetrain,
            m_intake,
            m_indexer,
            m_shooter
        );

        configureBindings();
        configurePathPlanner();
        configureDashboard();

        if (RobotConstants.isCompetition()) {
            m_vision.recordCaptures(true);
        }

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        // Every time our hub becomes inactive, we vibrate the controller and schedule a command
        // that vibrates the controller when we should return and when we should start shooting.
        new Trigger(() -> HubShiftUtil.isHubActive()).onFalse(
            Commands.sequence(
                Commands.runEnd(
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.5),
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.0)
                ).withTimeout(0.325),
                Commands.waitUntil(() -> HubShiftUtil.getTimeInCurrentShift().orElse(Seconds.zero()).in(Seconds) <= 9.0),
                Commands.runEnd(
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.5),
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.0)
                ).withTimeout(0.325),
                Commands.waitUntil(() -> HubShiftUtil.getTimeInCurrentShift().orElse(Seconds.zero()).in(Seconds) <= 3.0),
                Commands.runEnd(
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 1.0),
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.0)
                ).withTimeout(0.5)
            )
        );

        // Before endgame, we vibrate the controller when we should return/not contact robots climbing.
        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() <= 35.0).onTrue(
            Commands.sequence(
                Commands.startEnd(
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.5),
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.0)
                ).withTimeout(0.08),
                Commands.waitSeconds(0.06),
                Commands.startEnd(
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.5),
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.0)
                ).withTimeout(0.08),
                Commands.waitSeconds(0.06),
                Commands.startEnd(
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.5),
                    () -> m_driverController.setRumble(RumbleType.kLeftRumble, 0.0)
                ).withTimeout(0.08)
            )
        );

        // After every power-cycle, we re-zero on enable.
        CommandScheduler.getInstance().schedule(m_shooter.getTurretZeroRoutine().ignoringDisable(true));
    }

    public final void periodic() {
        m_field.setRobotPose(m_drivetrain.getEstimatedPose());
        SmartDashboard.putNumber("Alliance Shift", ShiftTimeUtil.getAllianceShiftTime());
        SmartDashboard.putNumber("Battery Voltage", RobotConstants.isReal() ?
            RobotController.getBatteryVoltage() : SimulatedBattery.getBatteryVoltage().in(Volts)
        );

        RobotVisualizer.updateComponents();
    }

    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));

        m_shooter.setDefaultCommand(new ShooterControlCommand(
            m_shooter,
            m_drivetrain::getEstimatedPose,
            m_drivetrain::getChassisSpeeds,
            m_driverController.rightBumper()
        ));

        m_driverController.x().whileTrue(Commands.run(
            () -> {
                var modules = m_drivetrain.getSwerveModules();

                for (var module : modules) module.setDriveFFAccel(0.0);

                // Using setDesiredState instead of setWheelAzimuth allows for angle optimization.
                modules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
                modules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
                modules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
                modules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
            },
            m_drivetrain
        ));

        // TODO: right bumper run kicker and flywheel reverse to unjam
        // TODO: add button to stop drive/azimuth motors to stop oscillation.
        m_driverController.leftTrigger().onTrue(Commands.runOnce(() -> {
            m_intake.setExtended(true);
            m_intake.setRunning(true);
        })).onFalse(Commands.runOnce(() -> {
            m_intake.setRunning(false);
        }));

        m_driverController.b().onTrue(Commands.runOnce(m_intake::toggleIntakeExtended));

        m_driverController.rightTrigger().onTrue(
            Commands.parallel(
                Commands.run(() -> {
                    m_shooter.setFlywheelVelocity(ShooterUtils.getPolynomialVelocityRoot(
                        ShotCalculator.getFuelVelocity()
                    ));
                    // This is really hacky but it should allow us to just override the flywheel
                    // while allowing the rest of the shooter control loop to run. The subsystem
                    // should probably be split up into its individual components instead of this.
                }, m_shooter.getFlywheel()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),

                Commands.sequence(
                    // TODO: Check velocity with timeout.
                    Commands.waitSeconds(0.925),
                    Commands.runOnce(() -> {
                        m_indexer.setRunning(true);
                        m_shooter.setKickerRunning(true);
                    })
                )
            ).until(m_driverController.rightTrigger().negate()).finallyDo(() -> {
                m_shooter.stopFlywheel();
                m_shooter.setKickerRunning(false);
                m_indexer.setRunning(false);
            })
        );

        if (!RobotConstants.isCompetition()) {
            m_programmerController.rightTrigger().whileTrue(new ControllerDriveCommand(m_programmerController, m_drivetrain));
            m_programmerController.a().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
            m_programmerController.x().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
        }

        if (RobotConstants.isSimulated()) {}
    }

    private final void configurePathPlanner() {
        PathPlannerLogging.setLogTargetPoseCallback(
            pose -> {
                m_field.getObject("target pose").setPose(pose);
                Logger.recordOutput("PathPlanner/TargetPose", pose);
            }
        );

        PathPlannerLogging.setLogActivePathCallback(
            poses -> m_field.getObject("path").setPoses(poses)
        );

        // TODO: Clean up auto commands.
        NamedCommands.registerCommand("StartIntaking", Commands.runOnce(() -> {
            m_intake.setExtended(true);
            m_intake.setRunning(true);
        }));

        NamedCommands.registerCommand("StopDriving", Commands.runOnce(() -> m_drivetrain.drive(new ChassisSpeeds(), false, false)));

        NamedCommands.registerCommand("StopIntaking", Commands.runOnce(() -> m_intake.setRunning(false)));

        NamedCommands.registerCommand("ShootQuick", Commands.parallel(
            Commands.run(() -> {
                m_shooter.setFlywheelVelocity(ShooterUtils.getPolynomialVelocityRoot(
                    ShotCalculator.getFuelVelocity()
                ));
            }),

            Commands.sequence(
                Commands.waitSeconds(0.925),
                Commands.runOnce(() -> {
                    m_indexer.setRunning(true);
                    m_shooter.setKickerRunning(true);
                })
            )
        ).until(() -> !DriverStation.isAutonomousEnabled()).finallyDo(() -> {
            m_shooter.stopFlywheel();
            m_indexer.setRunning(false);
            m_intake.setRunning(false);
            m_shooter.setKickerRunning(false);
        }));

        NamedCommands.registerCommand("Shoot", Commands.parallel(
            Commands.run(() -> {
                m_shooter.setFlywheelVelocity(ShooterUtils.getPolynomialVelocityRoot(
                    ShotCalculator.getFuelVelocity()
                ));
            }),

            Commands.sequence(
                Commands.waitSeconds(3.5),
                Commands.runOnce(() -> {
                    m_indexer.setRunning(true);
                    m_shooter.setKickerRunning(true);
                })
            )
        ).until(() -> !DriverStation.isAutonomousEnabled()).finallyDo(() -> {
            m_shooter.stopFlywheel();
            m_indexer.setRunning(false);
            m_intake.setRunning(false);
            m_shooter.setKickerRunning(false);
        }));

        NamedCommands.registerCommand("AutoFinish", Commands.runOnce(() -> {
            m_shooter.stopFlywheel();
            m_shooter.setKickerRunning(false);
            m_indexer.setRunning(false);
            m_intake.setRunning(false);
        }));

        NamedCommands.registerCommand("AutoFinished", Commands.runOnce(() -> {
            m_shooter.stopFlywheel();
            m_shooter.setKickerRunning(false);
            m_indexer.setRunning(false);
            m_intake.setRunning(false);
        }));

        // TODO: Clean up autos.
        m_autoChooser.addDefaultOption("None", Commands.none());
        m_autoChooser.addOption("Alliance Side", new PathPlannerAuto("Alliance Side", false));
        m_autoChooser.addOption("Left Self Pass", new PathPlannerAuto("self_pass", false));
        m_autoChooser.addOption("Right Self Pass", new PathPlannerAuto("self_pass", true));
        m_autoChooser.addOption("Shallow Self Pass Left", new PathPlannerAuto("shallow self pass", false));
        m_autoChooser.addOption("Shallow Self Pass Right", new PathPlannerAuto("shallow self pass", true));
        m_autoChooser.addOption("Alliance Side Full", new PathPlannerAuto("alliance side front"));
        m_autoChooser.addOption("Alliance Side Half", new PathPlannerAuto("alliance side half"));
        m_autoChooser.addOption("Broken Turret Left", new PathPlannerAuto("broken turret", false));
        m_autoChooser.addOption("Broken Turret Right", new PathPlannerAuto("broken turret", true));
        m_autoChooser.addOption("2.5swipe", new PathPlannerAuto("2.5swipe"));
        m_autoChooser.addOption("right_trench_3x", new PathPlannerAuto("right_trench_3x"));
    }

    private final void configureDashboard() {
        m_autoChooserAlert.set(true);
        m_autoChooser.getSendableChooser().onChange(
            key ->  m_autoChooserAlert.set(key.equals("None"))
        );

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    public final Command getAutonomousCommand() {
        return Objects.requireNonNullElse(m_autoChooser.get(), Commands.none());
    }
}
