package control;

import adapter.TrajectoryAdapter;
import adapter.TrajectorySegment;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;
import com.team254.lib.util.math.Twist2d;
import jaci.pathfinder.Trajectory;
import kinematics.DifferentialDriveKinematics;
import kinematics.Movement;
import kinematics.Pose;

import java.util.List;

/**
 * Implementation of a non-linear time-verying reference trajectory tracking controller.
 */
public class FeedforwardController implements IController {

    private List<TrajectorySegment> trajectory;
    private DifferentialDriveKinematics kinematics;

    private final double zeta, b;

    private Pose currentPose;
    private int currentSegmentIndex = 0;

    public FeedforwardController(double zeta, double b, DifferentialDriveKinematics kinematics) {
        this.zeta = zeta;
        this.b = b;
        this.kinematics = kinematics;
    }

    @Override
    public Movement update(Pose pose, double dt) {
        if(trajectory == null) {
            System.err.println("No trajectory specified");
            return new Movement(0.0, 0.0);
        }

        currentPose = pose;
        TrajectorySegment currentSegment = getGoalSegment();

        Twist2d output = calculateOutput(currentSegment.getVelocity(),  getTrackError(), getHeadingError(), currentPose.position.getRotation());

        if(!isFinished()) {
            currentSegmentIndex++;
        }

        return kinematics.inverseKinematics(output);
    }

    public Translation2d getTrackError() {
        return currentPose.position.getTranslation().translationTo(getGoalSegment().getPosition().getTranslation());
    }

    public Rotation2d getHeadingError() {
        return currentPose.position.getRotation().rotationTo(getGoalSegment().getPosition().getRotation());
    }

    private Twist2d calculateOutput(RigidTransform2d goalVelocity, Translation2d trackError, Rotation2d headingError, Rotation2d currentHeading) {

        double velocityGain = calculateVelocityGain(goalVelocity);
        double angularVelocityGain = calculateAngularVelocityGain(goalVelocity);

        double goalLinearVelocity = goalVelocity.getTranslation().norm();
        Rotation2d goalAngularVelocity = goalVelocity.getRotation();
        
        double linearVelocity = (goalLinearVelocity * headingError.cos()) + (velocityGain * (currentHeading.cos() * trackError.x() + currentHeading.sin() * trackError.y()));
        double angularVelocity = goalAngularVelocity.getRadians() + (angularVelocityGain * Math.signum(goalLinearVelocity) * ((currentHeading.cos() * trackError.y()) - (currentHeading.sin() * trackError.x()))) + (velocityGain * headingError.getRadians());
        
        return new Twist2d(linearVelocity, 0.0, angularVelocity);
    }

    private double calculateVelocityGain(RigidTransform2d goalVelocity) {
        double goalAngularVel = goalVelocity.getRotation().getRadians();
        double goalVel = goalVelocity.getTranslation().norm();
        return 2 * zeta * Math.sqrt((goalAngularVel * goalAngularVel) + (b * (goalVel * goalVel)));
    }

    private double calculateAngularVelocityGain(RigidTransform2d goalVelocity) {
        return b * Math.abs(goalVelocity.getTranslation().norm());
    }

    public TrajectorySegment getGoalSegment() {
        return trajectory.get(currentSegmentIndex);
    }

    public boolean isFinished() {
        return currentSegmentIndex >= trajectory.size() - 1;
    }

    public void reset() {
        currentSegmentIndex = 0;
    }

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = new TrajectoryAdapter(trajectory).getSegments();
        reset();
    }


}
