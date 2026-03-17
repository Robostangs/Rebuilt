package frc.robot.commands.DumperCommands.Flywheel;

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
       
        double targetrps =  75;
        turretShooter.setFlywheelVelocity(targetrps);
        // turretShooter.setMotionMagic();

        turretShooter.postStatus("Charging to " + targetrps + " rps Trench");
    }

    @Override
    public void end(boolean interrupted) {
        turretShooter.postStatus("Charged Trench");
        turretShooter.setFlywheelVelocity(0);
    }

}
