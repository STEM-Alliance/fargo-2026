package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModule;

public final class TuningCommands {
    public static final Command getCharacterizationRoutine(DrivetrainSubsystem drivetrain) {
        // set units to radians, set scaling to 1/12.6 (or gear ratio) & ctre 6 units plus velocity loop type
        // do 0.05 pos error and 4pi vel error 6v max input
        var driveMotorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(4.0), null,
                (state) -> Logger.recordOutput("SysIDState1", state.toString())
            ),
            new SysIdRoutine.Mechanism(drivetrain::setSwerveModulesDriveMotorVoltage, null, drivetrain)
        );

        var azimuthMotorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(4.0), null,
                (state) -> Logger.recordOutput("SysIDState2", state.toString())
            ),
            new SysIdRoutine.Mechanism(drivetrain::setSwerveModulesAzimuthMotorVoltage, null, drivetrain)
        );

        var routineCommand = Commands.sequence(
            Commands.runOnce(() -> drivetrain.setSwerveModulesWheelAzimuth(Radians.of(0.0))),
            Commands.waitSeconds(1.5),
            driveMotorRoutine.quasistatic(Direction.kForward).withDeadline(Commands.waitSeconds(4.0)),
            Commands.runOnce(() -> drivetrain.drive(new ChassisSpeeds(), false, false)),
            Commands.waitSeconds(1.5),
            driveMotorRoutine.quasistatic(Direction.kReverse).withDeadline(Commands.waitSeconds(4.0)),
            Commands.runOnce(() -> drivetrain.drive(new ChassisSpeeds(), false, false)),
            Commands.waitSeconds(1.5),
            driveMotorRoutine.dynamic(Direction.kForward).withDeadline(Commands.waitSeconds(1.0)),
            Commands.runOnce(() -> drivetrain.drive(new ChassisSpeeds(), false, false)),
            Commands.waitSeconds(1.5),
            driveMotorRoutine.dynamic(Direction.kReverse).withDeadline(Commands.waitSeconds(1.0)),
            Commands.runOnce(() -> drivetrain.drive(new ChassisSpeeds(), false, false)),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> drivetrain.setSwerveModulesWheelAzimuth(Radians.of(0.0))),
            Commands.waitSeconds(1.5),
            azimuthMotorRoutine.quasistatic(Direction.kForward).withDeadline(Commands.waitSeconds(1.0)),
            Commands.waitSeconds(1.5),
            azimuthMotorRoutine.quasistatic(Direction.kReverse).withDeadline(Commands.waitSeconds(1.0)),
            Commands.waitSeconds(1.5),
            azimuthMotorRoutine.dynamic(Direction.kForward).withDeadline(Commands.waitSeconds(0.5)),
            Commands.waitSeconds(1.5),
            azimuthMotorRoutine.dynamic(Direction.kReverse).withDeadline(Commands.waitSeconds(0.5)),
            Commands.waitSeconds(1.5)
        );

        routineCommand.addRequirements(drivetrain);

        return routineCommand;
    }

    public static final Command getWheelRadiusCommand(DrivetrainSubsystem drivetrain) {
        SwerveModule[] swerveModules = drivetrain.getSwerveModules();

        return Commands.sequence(
            Commands.runOnce(
                // this does not rotate 180deg/s (~120), could be an issue with the drivetrain?
                () -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, Math.PI / 4.0), false, false),
                drivetrain
            ),

            Commands.waitSeconds(0.5),

            Commands.run(
                () -> {
                    ChassisSpeeds currentSpeeds = drivetrain.getChassisSpeeds();
                    double driveMotorVelocities = 0.0;

                    for (SwerveModule module : swerveModules) {
                        driveMotorVelocities += module.getInputs().driveMotorVelocity.abs(RadiansPerSecond);
                    }

                    // temp
                    Logger.recordOutput("WheelDiameterMeters", ((currentSpeeds.omegaRadiansPerSecond * 6.12 * Math.hypot(0.276, 0.276)) / (driveMotorVelocities / 4.0)));
                },
                drivetrain
            )
        );
    }
}
