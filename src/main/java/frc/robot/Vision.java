package frc.robot;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;


import org.photonvision.PhotonCamera;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

 
 public class Vision {
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;

 
     
    public Vision(CommandSwerveDrivetrain drivetrain) {
        camera = new PhotonCamera(Constants.VisionConstants.kPhotonCamName);
        this.drivetrain = drivetrain;

    }
    Pose3d getPose(PhotonTrackedTarget target){
        Pose3d robotPose = new Pose3d();
        if (Constants.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), Constants.kAprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), Constants.VisionConstants.kRobotToCam);
            return robotPose;
        }else{
            return null;
        }
        
    }

     
    public void periodic(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            var newPose=getPose(target);
            if(newPose!=null){
                SmartDashboard.putNumber("Photon AprilTag Seen", target.getFiducialId());

                drivetrain.addVisionMeasurement(newPose.toPose2d(), result.getTimestampSeconds());
            }
        }

    }
 }



