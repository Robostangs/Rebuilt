package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision {
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final Transform3d robotToCam;
    PhotonPoseEstimator ppe;

    public PhotonVision(String photonCamName, Transform3d robotToCam) {
        camera = new PhotonCamera(photonCamName);
        this.drivetrain = CommandSwerveDrivetrain.getInstance();
        this.robotToCam = robotToCam;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
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
        if (yoPoseEsti.getFiducialId() == -1) {
            return false;
        }

        if (currentPose == null) {
            return false;
        }

        return true;
    }

    public void periodic() {

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = ppe.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = ppe.estimateLowestAmbiguityPose(result);
            }
            visionEst.ifPresent(
                    est -> {
                        CommandSwerveDrivetrain.getInstance().addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                        Robot.teleopField.getObject(camera.getName()+" Pose");

                    });
        
        }

       
    }
}
