package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;


public class setTurretDutyCycle extends Command {
	private final Turret turret;
	private final double dutyCycle;


	public setTurretDutyCycle(double dutyCycle) {
		this.dutyCycle = dutyCycle;
		this.turret = Turret.getInstance();
		addRequirements(turret);
	}

	@Override
	public void initialize() {
		turret.runTurretDutyCycle(dutyCycle);
		turret.postStatus("setTurretDutyCycle started: " + dutyCycle);
	}

	@Override
	public void end(boolean interrupted) {
		turret.runTurretDutyCycle(0.0);
		turret.postStatus("setTurretDutyCycle ended");
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
