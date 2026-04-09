package frc.robot.commands.DumperCommands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.FeederCommands.Feed;
import frc.robot.FeederCommands.setFeederSpeed;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.IntakeFactory;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Dumper.Flywheel;

public class DumpFactory {
        public static Command Shoot() {
                return new ChargeUp()
                                .alongWith(new setHoodPosition())
                                .alongWith(new RunHopper(() -> Constants.HopperConstants.kRollerDC))
                                .alongWith(new setFeederSpeed(() -> Constants.FeederConstants.kFeedDutyCycle));
        }

        public static Command SmartFeed() {
                return new DeferredCommand(
                                () -> {
                                        if (!Robot.isShooterDumb()) {

                                                return new WaitUntilCommand(
                                                                () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Feed(
                                                                                () -> 85))
                                                                .alongWith(new WaitUntilCommand(() -> Feeder
                                                                                .getInstance().feederAtTargetVelocity())
                                                                                .andThen(new RunHopper(
                                                                                                () -> Constants.HopperConstants.kRollerDC)));
                                        } else {
                                                return (new RunHopper(() -> Constants.HopperConstants.kRollerDC))
                                                                .alongWith(new setFeederSpeed(
                                                                                () -> 1));
                                        }
                                }, Set.of(Hopper.getInstance(), Feeder.getInstance()));

        }

        public static Command RegressionShoot() {

                return new ChargeUp()
                                .alongWith(
                                                new setHoodPosition())
                                .alongWith(
                                                SmartFeed())
                                .withName("Regression Shoot Command");
        }

        public static Command Pass() {
                return new ChargeUp(() -> Constants.FlywheelConstants.kPassSetpoint)
                                .alongWith(new setHoodPosition(
                                                () -> Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                .alongWith(
                                                SmartFeed())
                                .withName("Pass Command");
        }

        public static Command ElectroShoot() {
                return new ChargeUp(() -> Constants.FlywheelConstants.kDumbShootRPS)
                                .alongWith(
                                                new setHoodPosition(() -> Constants.HoodConstants.kDumbShootHoodPos))
                                .alongWith(
                                                SmartFeed())
                                .withName("Regression Shoot Command");
        }

        public static Command SmartShoot() {
                if (Robot.isPassing()) {
                        return Pass();
                } else {
                        return RegressionShoot().alongWith(IntakeFactory.OscillateIntake());
                }
        }

        public static Command PhotoShoot() {
                return (RegressionShoot());
                
                // .alongWith(IntakeFactory.OscillateIntake()))
                //                 .until(() -> Hopper.getInstance().getPhotoSensor());
        }

}