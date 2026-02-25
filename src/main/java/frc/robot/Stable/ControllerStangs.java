package frc.robot.Stable;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerStangs extends CommandXboxController {
    public ControllerStangs(int port) {
        super(port);
    }

    public Trigger c(){
        return this.povLeft();
    }
    public Trigger z(){
        return this.povRight();
    }
    public Trigger m1(){
        return this.leftStick();
    }
    public Trigger m2(){
        return this.rightStick();
    }
    public Trigger lm(){
        return this.start();
    }
    public Trigger rm(){
        return this.back();
    }
}
