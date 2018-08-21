package math.geometry;

import math.interpolation.Interpolable;

/**
 * Represents a change in position and velocity relative to a previous Pose
 */
public class Pose implements Interpolable<Pose> {

    public final RigidTransform2d position;
    public final RigidTransform2d velocity;
    public final RigidTransform2d acceleration;

    public Pose(RigidTransform2d position, RigidTransform2d velocity, RigidTransform2d acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public Pose(RigidTransform2d position, RigidTransform2d velocity) {
        this(position, velocity, new RigidTransform2d());
    }

    public Pose() {
        this(new RigidTransform2d(), new RigidTransform2d(), new RigidTransform2d());
    }

    public Pose update(Twist2d distanceTraveled, Twist2d velocity, double dt) {
        RigidTransform2d positionDelta = RigidTransform2d.exp(distanceTraveled);
        RigidTransform2d xyVelocity = RigidTransform2d.exp(velocity);
        RigidTransform2d xyAcceleration = RigidTransform2d.exp(new Twist2d(velocity.dx / dt, velocity.dy / dt, velocity.dtheta / dt));
        return new Pose(position.transformBy(positionDelta), xyVelocity, xyAcceleration);
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
