// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Turret.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Stable.ControllerStangs;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.IntakeFactory;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.commands.TurretCommands.TurretFactory;
import frc.robot.commands.TurretCommands.Hood.HomeHood;
import frc.robot.commands.TurretCommands.Hood.setHoodDutyCycle;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        public static boolean useVision = true;
        private double MaxSpeed = 1.0 * Constants.TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = Constants.TunerConstants.kAngularRateAt12Volts;

        // second
        // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final ControllerStangs xDrive = new ControllerStangs(0);
        private final ControllerStangs xManip = new ControllerStangs(1);
        private final GenericHID xSim = new GenericHID(2);
        private static RobotContainer mInstance;
        public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

        public static RobotContainer getInstance() {
                if (mInstance == null) {
                        mInstance = new RobotContainer();
                }
                return mInstance;
        }

        public RobotContainer() {
                configureDrivingBinding();
                configureManipBindings();
                configureSimBinds();
                drivetrain.registerTelemetry(logger::telemeterize);
                logger.initSwerveLog();
        }

        public Command controllerHaptics(double secs, CommandXboxController controller) {
                return new StartEndCommand(() -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
                }, () -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }).withDeadline(new WaitCommand(secs));
        }

        public static Command rumbleDriveController(double secs) {
                return RobotContainer.getInstance().controllerHaptics(secs, RobotContainer.getInstance().xDrive);
        }

        public static Command rumbleManipController(double secs) {
                return RobotContainer.getInstance().controllerHaptics(secs, RobotContainer.getInstance().xManip);
        }

        private void configureDrivingBinding() {
                setDrivetrain();

                xDrive.rightTrigger().whileTrue(TurretFactory.Shoot().alongWith(new RunHopper()))
                                .onFalse(new InstantCommand(() -> Hood.getInstance().setMotionMagicTarget(0),
                                                Hood.getInstance())
                                                .andThen(new InstantCommand(
                                                                () -> Hood.getInstance().setMotionMagic(),
                                                                Hood.getInstance())));

                xDrive.povDown().onTrue(
                                drivetrain.runOnce(() -> drivetrain.resetPose(
                                                !Robot.isRed() ? Constants.FieldConstants.resetPose
                                                                : FlippingUtil.flipFieldPose(
                                                                                Constants.FieldConstants.resetPose))));

                xDrive.povUp().onTrue(drivetrain.runOnce(
                                () -> drivetrain.seedFieldCentric(new Rotation2d(Units.degreesToRadians(180)))));

                xDrive.rightPaddle().whileTrue(new ExtendIntake(false));

                xDrive.leftPaddle().whileTrue(new RetractIntake());

                xDrive.rightBumper().whileTrue(IntakeFactory.TrueSpit());
                xDrive.leftBumper().whileTrue(IntakeFactory.ballStuck());

                xDrive.povRight().onTrue(new HomeHood());

                xDrive.povLeft().whileTrue(new HomeIntake(false));

                xDrive.x().whileTrue(new StartIntake());
                xDrive.b().onTrue(new InstantCommand(
                                (() -> useVision = !useVision)));

                xDrive.a().whileTrue(new AutoAlign(() -> xDrive.getLeftY(), () -> xDrive.getLeftX(),
                                () -> xDrive.getLeftTriggerAxis() > 0.2));

        }

        public void configureManipBindings() {
                DoubleSupplier leftStick = () -> (xManip.getLeftY() > 0.2) ? xManip.getLeftY() * 20 : 1;
                // xManip.y().onTrue(new InstantCommand(() -> Turret.setManipTarget(true)));

                // xManip.b().onTrue(new InstantCommand(() -> Turret.setManipTarget(false)));

 
                xManip.a().onTrue(new InstantCommand(() -> Hood.getInstance().setMM()));

                // xManip.b().onTrue(new InstantCommand(() -> Feeder.getInstance().forceTog()));

                xManip.y().whileTrue(new ExtendIntake(false));
                xManip.x().whileTrue(new ExtendIntake(true));

                xManip.rightTrigger().whileTrue(new setHoodDutyCycle(() -> 0.3));
                xManip.leftTrigger().whileTrue(new setHoodDutyCycle(() -> -0.3));

                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.05)
                                .whileTrue(new InstantCommand(
                                                () -> Flywheel.getInstance().adjustRPM(() -> xManip.getLeftY() * -30)));
        }

        private void setDrivetrain() {
                DoubleSupplier leftTrigger = () -> (xDrive.getLeftTriggerAxis() > 0.2) ? 0.25 : 1;
                if (Robot.isSimulation()) {
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(() -> drive
                                                        .withVelocityX((-xSim.getRawAxis(0))
                                                                        * MaxSpeed)
                                                        .withVelocityY((xSim.getRawAxis(1))
                                                                        * MaxSpeed)
                                                        .withRotationalRate((-xSim.getRawAxis(2))
                                                                        * MaxAngularRate)));
                } else {

                        drivetrain.setDefaultCommand(
                                        // Drivetrain will execute this command periodically
                                        drivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-xDrive.getLeftY() * MaxSpeed
                                                                        * leftTrigger.getAsDouble()) // Drive
                                                        // forward
                                                        // with
                                                        // negative
                                                        // Y
                                                        // (forward)
                                                        .withVelocityY(-xDrive.getLeftX() * MaxSpeed
                                                                        * leftTrigger.getAsDouble()) // Drive
                                                        // left
                                                        // with
                                                        // negative
                                                        // X
                                                        // (left)
                                                        .withRotationalRate(
                                                                        -xDrive.getRightX() * MaxAngularRate
                                                                                        * (leftTrigger.getAsDouble()
                                                                                                        + .25)) // Drive
                                        // counterclockwise
                                        // with
                                        // negative
                                        // X
                                        // (left)
                                        ));

                }
        }

        private void configureSimBinds() {
                new Trigger(() -> xSim.getRawButtonPressed(1))
                                .toggleOnTrue(new AutoAlign(() -> -xSim.getRawAxis(0),
                                                () -> xSim.getRawAxis(1),
                                                () -> false));

        }
}
