package kinematics;

import com.team254.lib.util.Interpolable;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Twist2d;

/**
 * Represents a change in position and velocity relative to a previous Pose
 */
public class Pose implements Interpolable<Pose> {

    public final RigidTransform2d position;
    public final RigidTransform2d velocity;

    public Pose(RigidTransform2d position, RigidTransform2d velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public Pose() {
        this.position = new RigidTransform2d();
        this.velocity = new RigidTransform2d();
    }

    public Pose update(Twist2d distanceTraveled, Twist2d velocity) {
        RigidTransform2d positionDelta = RigidTransform2d.exp(distanceTraveled);
        RigidTransform2d xyVelocity = RigidTransform2d.exp(velocity);
        return new Pose(position.transformBy(positionDelta), xyVelocity);
    }

    @Override
    public Pose interpolate(Pose other, double x) {
        return new Pose(position.interpolate(other.position, x), velocity.interpolate(other.velocity, x));
    }
}
