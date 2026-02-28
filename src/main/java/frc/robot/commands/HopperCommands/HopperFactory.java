package frc.robot.commands.HopperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TurretCommands.Flywheel.ChargeUp;
import frc.robot.commands.TurretCommands.Hood.setHoodPosition;

public class HopperFactory {

    public static Command StartHopper(){
        return 
            new StartBackGoer().alongWith(new StartLeftRighter());
    }
    
}
