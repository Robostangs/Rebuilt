package frc.robot.commands.DumperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.Index;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;

public class dumpFactory {
    public static Command Shoot(){

        return
        //  new InstantCommand(() -> Robot.setShooterIsDumb(false))
        //     .alongWith(
                new ChargeUp()
                // )
            .alongWith(new setHoodPosition())
            .alongWith(new Index(false));
    }

    public static Command dumbShoot(){
        return new InstantCommand(() -> Robot.setShooterIsDumb(true))
            .alongWith(new ChargeUp(75))
            .alongWith(new setHoodPosition())
            .alongWith(new Index(true))
            .finallyDo(() -> Robot.setShooterIsDumb(false));
    }
    
    public static Command dumbShootTrench(){
        return new ChargeUp().alongWith(new setHoodPosition(.75)).alongWith(new Index(true));
    }
}