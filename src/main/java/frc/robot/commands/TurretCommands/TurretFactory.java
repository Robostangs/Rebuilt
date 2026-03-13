package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Index;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.commands.TurretCommands.Hood.setHoodPosition;
import frc.robot.subsystems.Turret.Flywheel;
import frc.robot.subsystems.Turret.Turret;

public class TurretFactory {
    public static Command Shoot(){
        return 
            new ChargeUp(false).alongWith(new setHoodPosition(false)).alongWith(new Index(false));
    }

    public static Command dumbShoot(){
        return new ChargeUp(true).alongWith(new setHoodPosition(true)).alongWith(new Index(true));
    }
}