package frc.robot.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVision {
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    PhotonPoseEstimator ppe;

    public PhotonVision(String photonCamName, Transform3d robotToCam) {
        camera = new PhotonCamera(photonCamName);
        this.robotToCam = robotToCam;
        ppe = new PhotonPoseEstimator(Constants.kAprilTagFieldLayout, robotToCam);
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
