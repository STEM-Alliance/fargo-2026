package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleConfig;

public final class DrivetrainConfiguration {
    public static final double kDriveMotorRatio = 6.12;
    public static final double kAzimuthMotorRatio = 12.6;
    public static final Distance kWheelDiameter = Inches.of(4 - (4.115 - 4));

    // This speed is used to calculate both a translation velocity from controller inputs and to
    // desaturate the calculated module states; it can only be exceeded by directly setting a voltage.
    public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(4.0);
    public static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(0.5);

    public static final SwerveModuleConfig[] kModuleConfigurations = new SwerveModuleConfig[] {
        new SwerveModuleConfig(1, 2, 3, new Translation2d(0.276, 0.276)), // Front Left
        new SwerveModuleConfig(4, 5, 6, new Translation2d(0.276, -0.276)), // Front Right
        new SwerveModuleConfig(7, 8, 9, new Translation2d(-0.276, 0.276)), // Back Left
        new SwerveModuleConfig(10, 11, 12, new Translation2d(-0.276, -0.276)) // Back Right
    };

    public static final TalonFXConfiguration kDriveMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(0.067).withKI(0.000).withKD(0.000)
            .withKS(0.00).withKV(0.131).withKA(0.013)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(500.0))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(800.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        );

    public static final TalonFXConfiguration kAzimuthMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(41.49 / 2.0).withKI(0.000).withKD(1.141)
            .withKS(0.000).withKV(0.000).withKA(0.000) // Exclude feedforward.
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(25.0))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(125.0))
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40.0))
            .withSupplyCurrentLimit(Amps.of(40.0))
        ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
            .withContinuousWrap(true)
        );

    // clean up below
    public static final Pose2d kSimulationStartingPose = new Pose2d(7.0, 2.75, Rotation2d.fromDegrees(120.0));
    // maybe use static initializer like in camera sim io

    @SuppressWarnings("unchecked")
    public static final DriveTrainSimulationConfig kSimulationConfig = new DriveTrainSimulationConfig(
        Pounds.of(135.0), // Robot mass
        Meters.of(0.965), // Bumper length (x)
        Meters.of(0.855), // Bumper length (y)
        Meters.of(kModuleConfigurations[0].moduleTranslation().getX() * 2.0), // Drivetrain x
        Meters.of(kModuleConfigurations[0].moduleTranslation().getY() * 2.0), // Drivetrain y
        COTS.ofPigeon2(),
        COTS.ofMark4(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, 3)
    );

    public static final double kDriveMotorToWheelFactor = (Math.PI * kWheelDiameter.in(Meters)) / kDriveMotorRatio;
}
