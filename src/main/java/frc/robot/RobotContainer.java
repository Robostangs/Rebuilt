// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Turret.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Stable.ControllerStangs;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.MovePivot;
import frc.robot.commands.TurretCommands.TurretFactory;
import frc.robot.commands.TurretCommands.setTurretDutyCycle;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.commands.TurretCommands.Hood.HomeHood;
import frc.robot.commands.TurretCommands.Hood.setHoodDutyCycle;
import frc.robot.commands.TurretCommands.Hood.setHoodPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        public static boolean useVision = true;
        private double MaxSpeed = 1.0 * Constants.JabbaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
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
                drivetrain.registerTelemetry(logger::telemeterize);
                logger.initSwerveLog();
        }
        public Command controllerHaptics(double secs, CommandXboxController controller){
                return new StartEndCommand(() -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
                }, () -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }).withDeadline(new WaitCommand(secs));
        }

        public static Command rumbleDriveController(double secs){
                return RobotContainer.getInstance().controllerHaptics(secs, RobotContainer.getInstance().xDrive);
        }
    
        public static Command rumbleManipController(double secs){
                return RobotContainer.getInstance().controllerHaptics(secs, RobotContainer.getInstance().xManip);
        }
        
        private void configureDrivingBinding() {
                setDrivetrain();

                // xDrive.y().whileTrue(new Index());

                // xDrive.rightStick().toggleOnTrue(new ExtendIntake())
                // .onFalse(new RetractIntake());

                // xDrive.povRight().onTrue(new InstantCommand(
                // (() -> useVision = !useVision)));

                xDrive.povDown().onTrue(
                                drivetrain.runOnce(() -> drivetrain.resetPose(Constants.FieldConstants.resetPose)));

                xDrive.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // xDrive.a().onTrue(new InstantCommand(()
                // ->Hood.getInstance().setHoodPosition(0)));
                xDrive.x().whileTrue(TurretFactory.Shoot().alongWith(new RunHopper()));
                // xDrive.rightBumper().whileTrue(new RunHopper());
                xDrive.a().whileTrue(new ChargeUp());
                xDrive.y().whileTrue(new setHoodPosition());
                xDrive.b().whileTrue(new HomeHood());



        }

        public void configureManipBindings() {
                xManip.y().onTrue(new InstantCommand(() -> Turret.setManipTarget(true)));

                xManip.b().onTrue(new InstantCommand(() -> Turret.setManipTarget(false)));

                xManip.x().toggleOnTrue(new HomeHood());

                xManip.rightBumper().whileTrue(new setTurretDutyCycle(0.3));
                xManip.leftBumper().whileTrue(new setTurretDutyCycle(-0.3));

                new Trigger(() -> Math.abs(xManip.getRightY()) > 0.1)
                                .whileTrue(new MovePivot(() -> xManip.getRightY()));

                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.1)
                                .whileTrue(new setHoodDutyCycle(() -> xManip.getLeftY() / 2));


        }

        private void setDrivetrain() {
                double leftTrigger = ((xDrive.getLeftTriggerAxis() > 0.2) ? 0.25
                                : 1);
                if (Robot.isSimulation()) {
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(() -> drive.withVelocityX((-xSim.getRawAxis(0))
                                                        * MaxSpeed)
                                                        .withVelocityY((xSim.getRawAxis(1))
                                                                        * MaxSpeed)
                                                        .withRotationalRate((-xSim.getRawAxis(2))
                                                                        * MaxAngularRate)));
                } else {

                        drivetrain.setDefaultCommand(
                                        // Drivetrain will execute this command periodically
                                        drivetrain.applyRequest(() -> drive
                                                        .withVelocityX(-xDrive.getLeftY() * MaxSpeed * leftTrigger) // Drive
                                                        // forward
                                                        // with
                                                        // negative
                                                        // Y
                                                        // (forward)
                                                        .withVelocityY(-xDrive.getLeftX() * MaxSpeed * leftTrigger) // Drive
                                                        // left
                                                        // with
                                                        // negative
                                                        // X
                                                        // (left)
                                                        .withRotationalRate(
                                                                        -xDrive.getRightX() * MaxAngularRate
                                                                                        * leftTrigger) // Drive
                                        // counterclockwise
                                        // with
                                        // negative
                                        // X
                                        // (left)
                                        ));

                }
        }
}
