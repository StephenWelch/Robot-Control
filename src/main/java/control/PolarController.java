package control;

import adapter.PathAdapter;
import adapter.PathSegment;
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
 * Implementation of a polar-coordinate based posture stabilization controller.
 */
public class PolarController implements IController {

    private DifferentialDriveKinematics kinematics;

    private List<PathSegment> path;
    private int currentWaypointIndex = 0;
    private final double k1, k2, k3;

    public PolarController(double k1, double k2, double k3, DifferentialDriveKinematics kinematics) {
        this.k1 = k1;
        this.k2 = k2;
        this.k3 = k3;
        this.kinematics = kinematics;
    }

    @Override
    public Movement update(Pose pose, double dt) {
        if(path == null) {
            System.err.println("No trajectory specified.");
            return new Movement(0.0, 0.0);
        }
        RigidTransform2d waypoint = getGoalWaypoint();

        Translation2d trackError = waypoint.getTranslation().translationTo(pose.position.getTranslation());
        // p
        double distanceToGoal = trackError.norm();
        // y
        Rotation2d angleToGoal = Rotation2d.fromRadians(Math.atan2(trackError.y(), trackError.x()) - pose.position.getRotation().getRadians() + Math.PI);
        // o
        Rotation2d headingError = pose.position.getRotation().rotationTo(waypoint.getRotation());

//        System.out.println("P: " + distanceToGoal + " Y: " + angleToGoal + " O: " + headingError);

        double linearVelocity = k1 * distanceToGoal * angleToGoal.cos();
        double angularVelocity = (k2 * angleToGoal.getRadians()) + (k1 * ((angleToGoal.sin() * angleToGoal.cos()) / angleToGoal.getRadians())) * (angleToGoal.getRadians() + (k3 * headingError.getRadians()));

        if(!isFinished()) {
            currentWaypointIndex++;
        }

        return kinematics.inverseKinematics(new Twist2d(linearVelocity, 0.0, angularVelocity));
    }

    public void reset() {
        currentWaypointIndex = 0;
    }

    public void setPath(Trajectory trajectory) {
        this.path = new PathAdapter(trajectory).getSegments();
        reset();
    }

    public RigidTransform2d getGoalWaypoint() {
        return path.get(currentWaypointIndex).getPosition();
    }

    @Override
    public boolean isFinished() {
        return currentWaypointIndex >= path.size() - 1;
    }

}
