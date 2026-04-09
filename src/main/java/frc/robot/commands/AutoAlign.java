package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY;
    Supplier<Boolean> babyOnBoard;
    Supplier<Rotation2d> getTargetRotation;
    boolean isHub;
    boolean isAuto;

    Pose2d targetPose;

    public AutoAlign(Supplier<Double> translateX, Supplier<Double> translateY, Supplier<Boolean> babyOnBoard) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.translateX = translateX;
        this.translateY = translateY;
        this.babyOnBoard = babyOnBoard;
        this.isAuto = false;

        this.addRequirements(drivetrain);

    }

    public AutoAlign() {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.translateX = () -> 0D;
        this.translateY = () -> 0D;
        this.babyOnBoard = () -> false;
        this.isAuto = true;

        this.addRequirements(drivetrain);

    }

    @Override
    public void initialize() {

        PhoenixPIDController tunePID = new PhoenixPIDController(6.5, 0.5, 0.4);
        tunePID.enableContinuousInput(-Math.PI, Math.PI);

        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = tunePID;

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Auto Align PID", driveRequest.HeadingController);
        drivetrain.postStatus("Auto Aligning");

        driveRequest.Deadband = Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.05;
        driveRequest.RotationalDeadband = Constants.TunerConstants.kAngularRateAt12Volts * 0.05;
    }

    @Override
    public void execute() {

        targetPose = Robot.setTargetPose();

        if (!Robot.isPassing()) {

            getTargetRotation = () -> {

                return Rotation2d
                        .fromRadians((Math.atan2(
                                drivetrain.getState().Pose.getY() - targetPose.getY(),
                                drivetrain.getState().Pose.getX() - targetPose.getX())))
                        // .minus(Rotation2d.kZero)

                        .minus(!Robot.isRed() ? Rotation2d.k180deg : Rotation2d.kZero);
            };
     
        } else {
            getTargetRotation = () -> Rotation2d.k180deg;
        }

        driveRequest.TargetDirection = getTargetRotation.get();

               SmartDashboard.putNumber("Drive current theta",
                  (
                  Units.radiansToRotations(
                Math.atan2(
                                drivetrain.getState().Pose.getY() - targetPose.getY(),
                                drivetrain.getState().Pose.getX() - targetPose.getX())
                  ) 
                                ));

                   SmartDashboard.putNumber("Drive Target theta",
                  (
                  driveRequest.TargetDirection.getRadians()
                                ));


        driveRequest
                .withVelocityX(-translateX.get()
                        * (babyOnBoard.get() ? 1.0 : 0.25)
                        * (Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()))
                .withVelocityY(-translateY.get()
                        * (babyOnBoard.get() ? 1.0 : 0.25)
                        * Constants.TunerConstants.kSpeedAt12Volts.baseUnitMagnitude());

        drivetrain.setControl(driveRequest);
        drivetrain.postStatus("Currently aligning");

        SmartDashboard.putBoolean("Is pid at setpoint", driveRequest.HeadingController.atSetpoint());

    }


    

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned");
    }

    @Override
    public boolean isFinished() {
        // if (isAuto) {
        //     return (driveRequest.HeadingController.atSetpoint());
        // }

        return false;
    }
}
