package frc.robot.commands.DumperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.FeederCommands.Index;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.setKirbyWheels;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Dumper.Flywheel;

public class dumpFactory {
        public static Command Shoot() {

                return new ChargeUp()
                                .alongWith(new setHoodPosition())
                                .alongWith(new RunHopper(() -> Constants.HopperConstants.kRollerDC))
                                .alongWith(new Index(() -> Constants.FeederConstants.kFeedDutyCycle));
        }

        public static Command feed() {
                     return  new WaitUntilCommand( () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Index(() -> Constants.FeederConstants.kFeedDutyCycle))
                                                                .alongWith(new WaitUntilCommand(() -> Feeder
                                                                                .getInstance().feederAtTargetVelocity())
                                                                                .andThen(new RunHopper(() -> Constants.HopperConstants.kRollerDC)));
        }



        public static Command ShootHalforPass() {

                return new ChargeUp(() -> Constants.FlywheelConstants.kHalfSetpoint)
                                .alongWith(new setHoodPosition(() ->Constants.HoodConstants.kHalfSetpoint))
                                .alongWith(
                                                new WaitUntilCommand(
                                                                () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Index(() -> Constants.FeederConstants.kFeedDutyCycle))
                                                                .alongWith(new WaitUntilCommand(() -> Feeder
                                                                                .getInstance().feederAtTargetVelocity())
                                                                                .andThen(new RunHopper(() -> Constants.HopperConstants.kRollerDC))));
        }

        public static Command pass() {
                return new ChargeUp(() ->Constants.FlywheelConstants.kPassSetpoint)
                                .alongWith(new setHoodPosition(() ->Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                .alongWith(
                                                new WaitUntilCommand(
                                                                () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Index(() -> Constants.FeederConstants.kFeedDutyCycle))
                                                                .alongWith(new WaitUntilCommand(0.3)
                                                                                .andThen(new RunHopper(() -> Constants.HopperConstants.kRollerDC))));
        }

        public static Command PhotoShoot() {
                return ShootHalforPass().alongWith(new setKirbyWheels(() -> -1));
        }

        public static Command ShootTrench() {

                return new InstantCommand(() -> Robot.setShooterIsDumb(false))
                                .andThen(
                                                new ChargeUp(() ->Constants.FlywheelConstants.kTrenchSetpoint))
                                .alongWith(new setHoodPosition(() ->Constants.HoodConstants.kTrenchSetpoint))
                                .alongWith(new RunHopper(() -> Constants.HopperConstants.kRollerDC))
                                .alongWith(new Index(() -> Constants.FeederConstants.kFeedDutyCycle));
        }



}