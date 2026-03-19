package frc.robot.commands.TurretCommands.Flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Flywheel;

public class ChargeUpTrench extends Command {
    Flywheel turretShooter;

    public ChargeUpTrench( ) {
        turretShooter = Flywheel.getInstance();
        addRequirements(turretShooter);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
   
        //Hood: 0.38
       
            double targetrps =  75;
        

        turretShooter.setShooterRPS(targetrps);
        // turretShooter.setMotionMagic();


        turretShooter.postStatus("Charging to " + targetrps + " rps Trench");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged Trench");
        turretShooter.setShooterRPS(0);
    }

}
