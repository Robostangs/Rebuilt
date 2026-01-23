// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Repulsor;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    public static boolean useVision = true;
    private double MaxSpeed = 1.0 * Constants.ZiroTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                      // desired top
                                                                                                      // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xDrive = new CommandXboxController(0);
    private final CommandXboxController xManip = new CommandXboxController(1);
    private final GenericHID xSim = new GenericHID(2);

    public final CommandSwerveDrivetrain drivetrain = Constants.ZiroTunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        logger.initSwerveLog();

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        if (Robot.isSimulation()) {
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(() -> drive.withVelocityX((-xSim.getRawAxis(0))
                                                        * MaxSpeed
                                                                        )
                                                        .withVelocityY((xSim.getRawAxis(1))
                                                                        * MaxSpeed)
                                                        .withRotationalRate((-xSim.getRawAxis(2))
                                                                        * MaxAngularRate)));
                } else{
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-xDrive.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y
                                                                                                 // (forward)
                        .withVelocityY(-xDrive.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-xDrive.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));}

        xDrive.rightTrigger().whileTrue(new Repulsor());

        // Reset the field-centric heading on left bumper press.
        xDrive.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        xDrive.povRight().onTrue(new InstantCommand(
                (() -> useVision = !useVision)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
