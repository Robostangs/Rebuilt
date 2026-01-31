package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretShooter;

public class Repulsor extends Command {

    TurretShooter shooter;
    Turret turret;
    public Repulsor( ) {
        this.shooter = TurretShooter.getInstance();
        this.turret = Turret.getInstance();
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        shooter.setShooterRPS(0.1);
        //turret.setHappyDutyCycle(-0.25);
    }

    @Override
    public void execute(){
    }
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRPS(0);
        //turret.setHappyDutyCycle(0);
    }
}