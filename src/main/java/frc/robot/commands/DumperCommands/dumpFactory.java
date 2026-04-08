package frc.robot.commands.DumperCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Index;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.HopperCommands.RunHopper;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Dumper.Flywheel;
import frc.robot.subsystems.Dumper.Hood;

public class dumpFactory {
        public static Command Shoot() {

                return new ChargeUp()
                                .alongWith(new setHoodPosition())
                                .alongWith(new RunHopper())
                                .alongWith(new Index());
        }

        public static Command feed() {
                     return  new WaitUntilCommand( () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Index())
                                                                .alongWith(new WaitUntilCommand(() -> Feeder
                                                                                .getInstance().feederAtTargetVelocity())
                                                                                .andThen(new RunHopper()));
        }



        public static Command ShootHalforPass() {

                return new ChargeUp(() -> Constants.FlywheelConstants.kHalfSetpoint)
                                .alongWith(new setHoodPosition(() ->Constants.HoodConstants.kHalfSetpoint))
                                .alongWith(
                                                new WaitUntilCommand(
                                                                () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Index())
                                                                .alongWith(new WaitUntilCommand(() -> Feeder
                                                                                .getInstance().feederAtTargetVelocity())
                                                                                .andThen(new RunHopper())));
        }

        public static Command pass() {
                return new ChargeUp(() ->Constants.FlywheelConstants.kPassSetpoint)
                                .alongWith(new setHoodPosition(() ->Constants.HoodConstants.kPosAtOppositeAlliancePass))
                                .alongWith(
                                                new WaitUntilCommand(
                                                                () -> Flywheel.getInstance().shooterAtTargetVelocity())
                                                                .andThen(new Index())
                                                                .alongWith(new WaitUntilCommand(0.3)
                                                                                .andThen(new RunHopper())));
        }

        public static Command PhotoShoot() {
                return ShootHalforPass().alongWith(new StartIntake());
        }

        public static Command ShootTrench() {

                return new InstantCommand(() -> Robot.setShooterIsDumb(false))
                                .andThen(
                                                new ChargeUp(() ->Constants.FlywheelConstants.kTrenchSetpoint))
                                .alongWith(new setHoodPosition(() ->Constants.HoodConstants.kTrenchSetpoint))
                                .alongWith(new RunHopper())
                                .alongWith(new Index());
        }



}