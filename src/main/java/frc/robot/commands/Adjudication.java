package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IronMan;

public class Adjudication extends Command {

    IronMan punisher;

    public Adjudication( ) {
        this.punisher = IronMan.getInstance();
        addRequirements(punisher);
    }

    @Override
    public void initialize() {
            punisher.setPepperdutycycle(-0.4);
            punisher.setHappyDutyCycle(-0.25);

    }

    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted) {
        punisher.setPepperdutycycle(0);
        punisher.setHappyDutyCycle(0);
    }
}