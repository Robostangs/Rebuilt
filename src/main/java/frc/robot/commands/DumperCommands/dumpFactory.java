package frc.robot.commands.DumperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Index;
import frc.robot.commands.DumperCommands.Flywheel.ChargeUp;
import frc.robot.commands.DumperCommands.Hood.setHoodPosition;
import frc.robot.commands.DumperCommands.Hood.setHoodPositionTrench;

public class dumpFactory {
    public static Command Shoot(){
        return 
            new ChargeUp(false).alongWith(new setHoodPosition(false)).alongWith(new Index(false));
    }

    public static Command dumbShoot(){
        return new ChargeUp(true).alongWith(new setHoodPosition(true)).alongWith(new Index(true));
    }
    
    public static Command dumbShootTrench(){
        return new ChargeUp(true).alongWith(new setHoodPositionTrench()).alongWith(new Index(true));
    }
}