package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.photonvision.PhotonCamera;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final Transform3d robotToCam;

    public PhotonVision(CommandSwerveDrivetrain drivetrain, String photonCamName,Transform3d robotToCam) {
        camera = new PhotonCamera(photonCamName);
        this.drivetrain = drivetrain;
        this.robotToCam = robotToCam;
    }

    Pose3d getPose(PhotonTrackedTarget target) {
        Pose3d robotPose = new Pose3d();
        if (Constants.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                    Constants.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                    robotToCam);
            return robotPose;
        } else {
            return null;
        }

    }
    

    private boolean isPosegoo(PhotonTrackedTarget yoPoseEsti) {
        Pose3d currentPose = PhotonUtils.estimateFieldToRobotAprilTag(yoPoseEsti.getBestCameraToTarget(),
        Constants.kAprilTagFieldLayout.getTagPose(yoPoseEsti.getFiducialId()).get(),
        robotToCam);
        if( yoPoseEsti.getFiducialId() == -1) {
            return false;
        }
        if(currentPose == null){
            return false;
        }
        return true;
    }

    public void periodic() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            
            SmartDashboard.putNumber("periodic photon run",0 );

            PhotonTrackedTarget target = result.getBestTarget();
            if(isPosegoo(target)) {
            SmartDashboard.putNumber("pose is goo ", 4);
            var newPose = getPose(target);

            SmartDashboard.putNumber("Photon AprilTag Seen", target.getFiducialId());
            SmartDashboard.putNumber("periodic photon tag pose",newPose.toPose2d().getX() );

            drivetrain.addVisionMeasurement(newPose.toPose2d(), result.getTimestampSeconds());
            
        }
    }
    }
}
