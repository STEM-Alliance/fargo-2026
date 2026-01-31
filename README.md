# fargo-2026
Contains the robot project for FRC 7048 (Red River Rage)'s 2026 season.

## Setup
### Computer Software
The only software *required* to modify and deploy robot code is WPILib and the NI Game Tools.
- [WPILib Suite](https://github.com/wpilibsuite/allwpilib/releases/latest) for the base development environment, plus some additional software.
- [NI Game Tools](https://ni.com/en/support/downloads/drivers/download.frc-game-tools.html) for the Driver Station and misc. utilities; we don't use LabVIEW.
- [Phoenix Tuner](https://apps.microsoft.com/detail/9NVV4PWDW27Z) for interfacing with CTRE products (like our motors and encoders).
- [Pathplanner](https://github.com/mjansen4857/pathplanner/releases/latest) for creating autonomous paths and sequencing autonomous commands.
- [Git](https://git-scm.com/install) for keeping both the robot project and programming collaboration organized.

### Vendor Dependencies
This robot project requires:
- [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json) (right-click and copy link address)
- [CTRE-Phoenix 6](https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json) (right-click and copy link address)
- [PhotonLib](https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json) (right-click and copy link address)
- [MapleSim](https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/maple-sim.json) (right-click and copy link address)

which can be installed via either:
- the GUI in the righthand side of WPILib VSCode, or
- the command palette (`Ctrl + Shift + P`), by entering `WPILib: Manage Vendor Libraries` and online installing.

### Building and Deploying
This year, we are using AdvantageKit, and parts of the robot project depend on files it generates during builds; when you first open the project, there will be missing classes that will be resolved once the project has been built.

To build the project, enter `WPILIB: Build Robot Code` into the command palette.

Once your project has been built (and deploying will also automatically build if needed), you can connect to the robot—either over ethernet or over WiFi to the radio—and upload your code to be ran on the RoboRIO.

To deploy the project, enter `WPILIB: Deploy Robot Code` into the command palette.

### Simulating
Simulation is something new that we are trying for this year. The code structure required for AdvantageKit makes it easy to create simulation implementations of our real IO interfaces so that the exact same code can be ran in simulation as in reality. The only difference is that it is up to our simulation implementations to process those inputs realistically.

Simulation is valuable in that you can rapidly test new code without deploying (or having to have the robot, set it up, or worry about damage), multiple programmers can simultaneously work on the "same robot," and that you can start programming before you even have hardware. 

You shouldn't initially assume that your simulation exactly mirrors reality and that there will be no issues with integration, but once you can roughly validate your simulation, it is likely that if it works in simulation, it will work on the real robot as well. In the case of our drivetrain, MapleSim is accurate enough that SysID gains that were derived from characterization in simulation functioned "good enough" on the real robot as well.

To simulate the project, enter `WPILIB: Simulate Robot Code` into the command palette and connect with AdvantageScope. Open with the SimGUI, and from there: you can connect driver controllers, set your alliance color, and set the state of the robot.

## Project Structure
As stated above, we are using AdvantageKit, which requires a different (but similar) code structure than previously. Instead of individual subsystems containing their own instances of their hardware, subsystems now contain (passed via a constructor) one or many IO interfaces and use those to access their hardware.

Being able to change where subsystems get their values from means that we can easily run the same code both in reality and in simulation using a switch statement (to switch between real and simulated IO interface implementations) like is done now in the `RobotContainer`.

Below is an example of a FlywheelIO interface:

```java
public abstract interface FlywheelIO {
    @AutoLog
    public static abstract class FlywheelInputs {
        // You will have more inputs than this. All of these
        // values are logged, which makes debugging a lot easier.
        public boolean isFlywheelMotorConnected = false;
        public Angle flywheelMotorPosition = Radians.of(0.0);
    }

    // A similar method is required for all IO interfaces.
    public default void updateInputs(FlywheelInputs loggableInputs) {}

    // Any methods for interacting with the hardware should contain
    // as little logic (and room for bugs) as possible; all of that
    // complexity should be contained within the subsystem instead.
    public default void setFlywheelVelocity(AngularVelocity velocity) {}
}
```

And its accompying FlywheelSubsystem:

```java
public final class FlywheelSubsystem {
    private final FlywheelIO m_flywheelIO;
    private final FlywheelInputsAutoLogged m_flywheelInputs = new FlywheelInputsAutoLogged();

    public final void periodic() {
        m_flywheelIO.updateInputs(m_flywheelInputs);
        Logger.processInputs("FlywheelSubsystem", m_flywheelInputs);
    }

    public final void setFlywheelDistance(Distance distance) {
        // An example of additional "complexity" in the subsystem.
        AngularVelocity velocity = FlywheelUtil.getVelocity(distance);
        m_flywheelIO.setFlywheelVelocity(velocity);
    }

    public final boolean isShooterMotorConnected() {
        return m_shooterInputs.isShooterMotorConnected;
    }
}
```

The `FlywheelInputsAutoLogged` class is automatically generated on build via the `@AutoLog` annotation. The parent `FlywheelInputs` class should be abstract to prevent accidental instantiation of itself; the program will still run, but no logging will be done.

This subsystem is so only in function; as it is only a flywheel (and would be part of a larger `ShooterSubsystem`), it is a "pseudo-subsystem" and its periodic method must be called by its "parent" subsystem, allowing for a defined update order of individual components and for keeping the parent subsystem more organized.

Similarly for allowing a defined update order of individual components, parent and/or standalone subsystems also implement the `Subsystem` interface instead of extending the `SubsystemBase` class for the same reason. Normally the update order would depend on instantiation, but there are some cases (like with drivetrain and vision subsystems) where the one that should update first needs an object from the other. In the worst case, subclassing results in some measurements being effectively two loops behind, but it is still a slight improvement for its complexity (you must manually register subsystems like is done in the `RobotContainer` now).
