package frc.robot.commands.TurretCommands.Flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Flywheel;
import frc.robot.subsystems.Turret.Turret;

public class ChargeUp extends Command {
    Flywheel turretShooter;
    boolean isDumb;

    public ChargeUp(boolean isDumb) {
        turretShooter = Flywheel.getInstance();
        this.isDumb = isDumb;
        addRequirements(turretShooter);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double meterfromHub = Turret.getInstance().getMetersFromHub();
            double targetrps = 0;
        if(!isDumb){
         targetrps = turretShooter.getTargetVelocity(meterfromHub);}
        else{
            targetrps =  75;
        }

        turretShooter.setShooterRPS(targetrps);
        // turretShooter.setMotionMagic();

        SmartDashboard.putNumber("Hood MFH", meterfromHub);

        turretShooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged");
        turretShooter.setShooterRPS(0);
    }

}
