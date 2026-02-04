// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Turret.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TurretCommands.HomeHood;
import frc.robot.commands.TurretCommands.SetHoodDutyCycle;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        public static boolean useVision = true;
        private double MaxSpeed = 1.0 * Constants.JabaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                          // desired top
                                                                                                          // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController xDrive = new CommandXboxController(0);
        private final CommandXboxController xManip = new CommandXboxController(1);
        private final GenericHID xSim = new GenericHID(2);

        public final CommandSwerveDrivetrain drivetrain = Constants.JabaTunerConstants.createDrivetrain();

        public RobotContainer() {
                configureDrivingBinding();
                configureManipBindings();
                logger.initSwerveLog();

        }

        private void configureDrivingBinding() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                if (Robot.isSimulation()) {
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(() -> drive.withVelocityX((-xSim.getRawAxis(0))
                                                        * MaxSpeed)
                                                        .withVelocityY((xSim.getRawAxis(1))
                                                                        * MaxSpeed)
                                                        .withRotationalRate((-xSim.getRawAxis(2))
                                                                        * MaxAngularRate)));
                } else {

                        DoubleSupplier leftY = () -> xDrive.getLeftY();
                        DoubleSupplier leftX = () -> xDrive.getLeftX();
                        DoubleSupplier rightX = () -> xDrive.getRightX();
                        
                        if (((Math.abs(leftY.getAsDouble()) > 0.05)
                                        || Math.abs(leftX.getAsDouble()) > 0.05
                                        || Math.abs((rightX.getAsDouble())) > 0.05)) {

                                drivetrain.setDefaultCommand(
                                                // Drivetrain will execute this command periodically
                                                drivetrain.applyRequest(() -> drive
                                                                .withVelocityX(-xDrive.getLeftY() * MaxSpeed) // Drive
                                                                                                              // forward
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // Y
                                                                                                              // (forward)
                                                                .withVelocityY(-xDrive.getLeftX() * MaxSpeed) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                                                                .withRotationalRate(
                                                                                -xDrive.getRightX() * MaxAngularRate) // Drive
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                                                ));
                        } else {
                                drivetrain.setDefaultCommand(
                                                drivetrain.applyRequest(() -> brake));
                        }
                }

                // Reset the field-centric heading on left bumper press.
                xDrive.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
                // xDrive.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new
                // Pose2d(3.6, 3.04, drivetrain.getRotation3d().toRotation2d()))));

                xDrive.povRight().onTrue(new InstantCommand(
                                (() -> useVision = !useVision)));

                drivetrain.registerTelemetry(logger::telemeterize);
                xDrive.x().onTrue(new InstantCommand(() -> TurretShooter.getInstance().setShooterRPS(40d)));
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

        public void configureManipBindings() {
                // Turret target hub
                // xManip.y().onTrue(new InstantCommand(() -> Turret.setManipTarget(true)));
                // Turret target pass
                // xManip.b().onTrue(new InstantCommand(() -> Turret.setManipTarget(false)));

                // new Trigger(() -> xManip.getLeftY() > 0.1 || xManip.getLeftY() < -0.1)
                // .whileTrue(new InstantCommand(
                // () -> Turret.getInstance().setTurretSetpointDoubleSupplier(
                // () -> xManip.getLeftY())));

                new Trigger(() -> Math.abs(xManip.getRightX()) > 0.1)
                                .whileTrue(new InstantCommand(() -> Turret.getInstance()
                                                .setIronmansetpoint(xManip.getRightX() / 10)));
                xManip.b().toggleOnTrue(new HomeHood());
                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.1)
                                .whileTrue(new SetHoodDutyCycle(() -> xManip.getLeftY() / 2));

        }
}
