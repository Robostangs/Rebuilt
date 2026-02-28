package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.commands.TurretCommands.Hood.setHoodPosition;

public class TurretFactory {
    public static Command Shoot(){
        return 
            new ChargeUp().alongWith(new setHoodPosition());
    }
}