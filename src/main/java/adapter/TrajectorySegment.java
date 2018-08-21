package adapter;

import math.geometry.RigidTransform2d;

public class TrajectorySegment {

    private RigidTransform2d position, velocity, acceleration, jerk;
    private double dt;

    public TrajectorySegment(RigidTransform2d position, RigidTransform2d velocity, RigidTransform2d acceleration, RigidTransform2d jerk, double dt) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
        this.dt = dt;
    }

    public RigidTransform2d getPosition() {
        return position;
    }

    public RigidTransform2d getVelocity() {
        return velocity;
    }

    public RigidTransform2d getAcceleration() {
        return acceleration;
    }

    public RigidTransform2d getJerk() {
        return jerk;
    }

    public double getDt() {
        return dt;
    }
}
