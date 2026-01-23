package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IronMan;

public class Repulsor extends Command {

    IronMan ironMan;

    public Repulsor( ) {
        this.ironMan = IronMan.getInstance();
        addRequirements(ironMan);
    }

    @Override
    public void initialize() {
            ironMan.setPepperdutycycle(-0.4);
            ironMan.setHappyDutyCycle(-0.25);

    }

    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted) {
        ironMan.setPepperdutycycle(0);
        ironMan.setHappyDutyCycle(0);
    }
}