package frc.robot.commands.DumperCommands.Flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Flywheel;
import frc.robot.Robot;

public class ChargeUp extends Command {
    Flywheel dumpShooter;
    boolean isDumb;

    public ChargeUp(boolean isDumb) {
        dumpShooter = Flywheel.getInstance();
        this.isDumb = isDumb;
        addRequirements(dumpShooter);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //error here is intentional for now. kabir add ts to robot.java por favor
        //should be done tn and then we can get rid of other unnecessary johns in turret
        double meterfromHub = Robot.getInstance().getMetersFromHub();
            double targetrps = 0;
        if(!isDumb){
         targetrps = dumpShooter.getTargetVelocity(meterfromHub);}
        else{
            targetrps =  75;
        }

        dumpShooter.setShooterRPS(targetrps);

        SmartDashboard.putNumber("Hood MFH", meterfromHub);

        dumpShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        dumpShooter.postStatus("Charged");
        dumpShooter.setShooterRPS(0);
    }

}
