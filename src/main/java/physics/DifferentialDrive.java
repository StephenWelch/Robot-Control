package physics;

import math.geometry.Rotation2d;
import math.geometry.Twist2d;

public class DifferentialDrive {

    private final double kEps = 1E-9;

    public final double WHEELBASE, SCRUB_FACTOR, WHEEL_DIAM;
    public final int TICKS_PER_ROTATION;

    public final double MASS, MOI;
    public final double ANGULAR_DRAG;

    public final DCMotorModel leftMotor, rightMotor;

    public DifferentialDrive(double WHEELBASE, double SCRUB_FACTOR, double WHEEL_DIAM, int TICKS_PER_ROTATION, double MASS, double MOI, double ANGULAR_DRAG, DCMotorModel leftMotor, DCMotorModel rightMotor) {
        this.WHEELBASE = WHEELBASE;
        this.SCRUB_FACTOR = SCRUB_FACTOR;
        this.WHEEL_DIAM = WHEEL_DIAM;
        this.TICKS_PER_ROTATION = TICKS_PER_ROTATION;
        this.MASS = MASS;
        this.MOI = MOI;
        this.ANGULAR_DRAG = ANGULAR_DRAG;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public Twist2d forwardKinematics(double leftDelta, double rightDelta, Rotation2d angleDelta) {
        return new Twist2d((leftDelta + rightDelta) / 2, 0, angleDelta.getRadians());
    }

    public Twist2d forwardKinematics(double leftDelta, double rightDelta) {
        double delta_v = rightDelta - leftDelta;
        Rotation2d delta_rotation = Rotation2d.fromRadians(delta_v / (WHEELBASE * SCRUB_FACTOR));
        return forwardKinematics(leftDelta, rightDelta, delta_rotation);
    }

    public Movement inverseKinematics(Twist2d twist) {
        if(Math.abs(twist.dx) < kEps) {
            return new Movement(twist.dx, twist.dx);
        }
        double delta_v = WHEELBASE * twist.dtheta / (2 * SCRUB_FACTOR);
        return new Movement(twist.dx - delta_v, twist.dx + delta_v);
    }

    public Twist2d forwardDynamics(Movement voltages, Movement velocities) {

        double leftAccel = leftMotor.getAcceleration(voltages.left, velocities.left);
        double rightAccel = rightMotor.getAcceleration(voltages.right, velocities.right);

        Twist2d accel = forwardKinematics(leftAccel, rightAccel);
        Twist2d vel = forwardKinematics(velocities.left, velocities.right);

        double effectiveAngularForce = (MOI * accel.dtheta) - (ANGULAR_DRAG * vel.dtheta);
        double effectiveLinearForce = MASS * accel.dx;

        return new Twist2d(effectiveLinearForce / MASS, 0.0, effectiveAngularForce / MOI);

    }

    /**
     * Accounts for angular drag from wheelbase scrub.
     * @param velocity The desired robot velocity.
     * @param acceleration The desired robot acceleration.
     * @return The left and right voltages needed to achieve the given velocity and acceleration.
     */
    public Movement inverseDynamics(Twist2d velocity, Twist2d acceleration) {
        double effectiveAngularForce = (acceleration.dtheta * MOI) + (ANGULAR_DRAG * velocity.dtheta);
        double effectiveLinearForce = acceleration.dx * MASS;
        Twist2d effectiveAcceleration = new Twist2d(effectiveLinearForce / MASS, 0.0, effectiveAngularForce / MOI);

        return inverseDynamics(inverseKinematics(velocity), inverseKinematics(effectiveAcceleration));
    }

    private Movement inverseDynamics(Movement velocities, Movement accelerations) {

        double leftVoltage = leftMotor.getVoltage(velocities.left, accelerations.left);
        double rightVoltage = rightMotor.getVoltage(velocities.right, accelerations.right);

        return new Movement(leftVoltage, rightVoltage);

    }

    public double ticksToRotations(int ticks) {
        return ticks / TICKS_PER_ROTATION;
    }

    public double ticksToDistance(int ticks) {
        return ticksToRotations(ticks) * (Math.PI * WHEEL_DIAM);
    }

    public double ticksToVel(int ticks) {
        return ticksToRotations(ticks) * (Math.PI * WHEEL_DIAM) * 10;
    }

}
