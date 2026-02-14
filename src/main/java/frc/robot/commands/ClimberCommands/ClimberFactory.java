package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.commands.ClimberCommands.*;

public class ClimberFactory{
    public enum climberStates{ L0, L1, L2, L3, stow}
    
    public static climberStates ClimberState = climberStates.stow;
    static Alert stowAlert = new Alert("Cannot go to stow", Alert.AlertType.kError);
    static Alert pivotAlert = new Alert("Cannot pivot", Alert.AlertType.kError);
    
    
    // public static 
    // public static Command L0Position(){
    //     return new DeferredCommand(() -> {
    //          if (
    //     }
    // }

        // public static Command L1Position(){
        // }
        // public static Command L2Position(){}

        // public static Command L3Position(){}

        // public static Command stowed(){}


}


