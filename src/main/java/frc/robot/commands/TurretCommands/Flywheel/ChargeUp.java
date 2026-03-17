package frc.robot.commands.TurretCommands.Flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Turret.Flywheel;
import frc.robot.subsystems.Turret.Turret;

public class ChargeUp extends Command {
    Flywheel shooter;

    public ChargeUp() {
        shooter = Flywheel.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double targetrps = shooter.getFlywheelTargetVelocity();
        shooter.setFlywheelVelocity(targetrps);

        SmartDashboard.putNumber("Hood MFH", Robot.getMetersFromTarget());

        shooter.postStatus("Charging to " + targetrps + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.postStatus("Charged");
        shooter.setFlywheelVelocity(0);
    }

}
