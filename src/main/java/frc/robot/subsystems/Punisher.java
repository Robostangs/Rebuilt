package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

   /**
     * The turret
     */
public class Punisher extends SubsystemBase {

    TalonFX punisherMotor;
    TalonFX jigsaswMotor;
    TalonFX microMotor;
    
    private static Punisher mInstance;

    /**
     * The turret
     */
    public Punisher() {
        punisherMotor = new TalonFX(Constants.PunisherConstants.punisherMotorID);
    }

       public static Punisher getInstance() {
        if (mInstance == null) {
            mInstance = new Punisher();
        }
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Punisher/status", status);
    }

    public void setPunisherdutycycle(double speed) {
        punisherMotor.set(speed);
        jigsaswMotor.set(speed);
        microMotor.set(speed);
    }
    
}
