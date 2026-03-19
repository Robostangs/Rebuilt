package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Index;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.commands.TurretCommands.Hood.setHoodPosition;
import frc.robot.commands.TurretCommands.Hood.setHoodPositionTrench;
import frc.robot.subsystems.Turret.Flywheel;

public class TurretFactory {
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