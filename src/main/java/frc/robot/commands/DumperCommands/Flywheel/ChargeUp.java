package frc.robot.commands.DumperCommands.Flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Dumper.Flywheel;

public class ChargeUp extends Command {
    Flywheel shooter;
    double targetRPS;
    boolean smort;


    public ChargeUp() {
        shooter = Flywheel.getInstance();
        addRequirements(shooter);
        targetRPS = shooter.getFlywheelTargetVelocity();
        smort = true;
    }    
    public ChargeUp(double targetRPS) {
        shooter = Flywheel.getInstance();
        addRequirements(shooter);
        this.targetRPS = targetRPS;
        smort = false;
    }

    @Override
    public void initialize() {
        shooter.setCanShoot(true);
    }

    @Override
    public void execute() {


        if(smort) {
            targetRPS = shooter.getFlywheelTargetVelocity();
        }

        shooter.setFlywheelVelocity(targetRPS);

        SmartDashboard.putNumber("Flywheel/Meters", Robot.getMetersFromTarget());

        shooter.postStatus("Charging to " + targetRPS + " rps");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.postStatus("Charged");
        shooter.setFlywheelVelocity(0);
    }

}