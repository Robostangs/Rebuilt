// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Stable.ControllerStangs;
import frc.robot.commands.DumperCommands.dumpFactory;
import frc.robot.commands.DumperCommands.Hood.HomeHood;
import frc.robot.commands.DumperCommands.Hood.setHoodDutyCycle;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.ExtendIntake;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.IntakeFactory;
import frc.robot.commands.IntakeCommands.RetractIntake;
import frc.robot.commands.IntakeCommands.SetPivotPosition;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Index;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dumper.Flywheel;
import frc.robot.subsystems.Dumper.Hood;

public class RobotContainer {
        public static boolean useVision = true;
        private double MaxSpeed = Constants.TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
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
        private boolean isCappingSpeed = false;

        public static RobotContainer getInstance() {
                if (mInstance == null) {
                        mInstance = new RobotContainer();
                }
                return mInstance;
        }

        public RobotContainer() {
                configureDrivingBinding();
                configureManipBindings();
                if (Robot.isSimulation()) {
                        configureSimBinds();

                }
                drivetrain.registerTelemetry(logger::telemeterize);
                logger.initSwerveLog();
        }

        private Command rumbleController(double secs, CommandXboxController controller) {
                return new StartEndCommand(() -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
                }, () -> {
                        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                }).withDeadline(new WaitCommand(secs));
        }

        public static Command rumbleDriveController(double secs) {
                return RobotContainer.getInstance().rumbleController(secs, RobotContainer.getInstance().xDrive);
        }

        public static Command rumbleManipController(double secs) {
                return RobotContainer.getInstance().rumbleController(secs, RobotContainer.getInstance().xManip);
        }

        private void configureDrivingBinding() {
                setDrivetrain();

                xDrive.rightTrigger().whileTrue(
                                (dumpFactory.ShootHalforPass()));
                // close shoot

                xDrive.rm()
                                .whileTrue(new AlignToHub().getAlignToHub(Constants.FieldConstants.kMeterToHalfBump));

                xDrive.lm()
                                .whileTrue(new AlignToHub().getAlignToHub(Constants.FieldConstants.kMeterToTower));

                xDrive.rightPaddle().whileTrue(new ExtendIntake());
                xDrive.leftPaddle().whileTrue(new RetractIntake());

                // TODO Find ts pose
                xDrive.povDown().onTrue(
                                drivetrain.runOnce(() -> drivetrain.resetPose(
                                                !Robot.isRed() ? Constants.FieldConstants.resetPose
                                                                : FlippingUtil.flipFieldPose(
                                                                                Constants.FieldConstants.resetPose))));

                xDrive.povUp().onTrue(drivetrain.runOnce(
                                () -> drivetrain.seedFieldCentric()));

                xDrive.povRight().onTrue(new InstantCommand(
                                (() -> useVision = !useVision)));

                xDrive.x().whileTrue(new StartIntake());


                xDrive.a().whileTrue(new AutoAlign(() -> xDrive.getLeftY(), () -> xDrive.getLeftX(),
                                () -> xDrive.getLeftTriggerAxis() < 0.2));

                xDrive.y().whileTrue(IntakeFactory.TrueSpit());
                xDrive.b().whileTrue(IntakeFactory.ballStuck());

        }

        public void configureManipBindings() {
                xManip.b().onTrue(
                                new InstantCommand(() -> Robot.toggleShooterPass()));

                xManip.y().onTrue(new InstantCommand(() -> Hood.getInstance().setMM()));

                xManip.a().whileTrue(new HomeHood());

                xManip.rightPaddle().whileTrue(new HomeIntake(false));
                xManip.leftPaddle().whileTrue(new HomeIntake(true));

                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.05)
                                .whileTrue(new InstantCommand(
                                                () -> Flywheel.getInstance().adjustRPM(() -> xManip.getLeftY() * -15)))
                                .onFalse(
                                                new InstantCommand(
                                                                () -> Flywheel.getInstance().adjustRPM(() -> 0)));

                xManip.rightTrigger().whileTrue(new setHoodDutyCycle(() -> xManip.getLeftTriggerAxis() * -0.2));

                xManip.leftTrigger().whileTrue(new setHoodDutyCycle(() -> xManip.getLeftTriggerAxis() * 0.2));

        }

        private void setDrivetrain() {
                DoubleSupplier leftBumper = () -> (xDrive.leftTrigger().getAsBoolean()) ? 0.5 : 1;

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
                                                                        * leftBumper.getAsDouble())
                                                        .withVelocityY(-xDrive.getLeftX() * MaxSpeed
                                                                        * leftBumper.getAsDouble())
                                                        .withRotationalRate(
                                                                        -xDrive.getRightX() * MaxAngularRate
                                                                                        * (leftBumper.getAsDouble()
                                                                                                        + .25))));
                }
        }

        private void configureSimBinds() {
                new Trigger(() -> xSim.getRawButtonPressed(1))
                                .whileTrue(new AutoAlign(() -> -xSim.getRawAxis(0),
                                                () -> xSim.getRawAxis(1),
                                                () -> true));

                new Trigger(() -> xSim.getRawButtonPressed(2))
                                .onTrue(new AlignToHub().getAlignToHub(4));

        }
}
