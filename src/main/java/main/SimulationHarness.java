package main;

import control.IController;
import math.geometry.RigidTransform2d;
import physics.DifferentialDrive;
import physics.Movement;

public class SimulationHarness {

    private IController controller;
    private PoseTracker poseTracker;
    private DifferentialDrive driveModel;

    public SimulationHarness(IController controller, PoseTracker poseTracker, DifferentialDrive driveModel) {
        this.controller = controller;
        this.poseTracker = poseTracker;
        this.driveModel = driveModel;
    }

    public PoseTracker simulate(double dt, double maxTime) {
        return simulate(dt, maxTime, new RigidTransform2d());
    }

    public PoseTracker simulate(double dt, double maxTime, RigidTransform2d initialPosition) {

        poseTracker.reset(initialPosition, 0.0);
        Movement totalDisplacement = new Movement(0.0, 0.0);
        Movement lastVelocities = new Movement(0.0, 0.0);

        for(double time = 0.0; !controller.isFinished(); time += dt) {
            Movement velocity = controller.update(poseTracker.getLatest(), 0.0);
            Movement accel = new Movement((velocity.left - lastVelocities.left) / dt, (velocity.right - lastVelocities.right) / dt);

            Movement voltages = driveModel.inverseDynamics(driveModel.forwardKinematics(velocity.left, velocity.right),
                                                            driveModel.forwardKinematics(accel.left, accel.right));

            voltages = Movement.clamp(voltages, 12.0, 12.0);
            voltages = new Movement(-voltages.left, -voltages.right);

            Movement simAccel = driveModel.inverseKinematics(driveModel.forwardDynamics(voltages, velocity));

            totalDisplacement = totalDisplacement.add(new Movement(0.5 * simAccel.left * dt * dt, 0.5 * simAccel.right * dt * dt));
//            totalDisplacement = totalDisplacement.add(new Movement(velocity.left * dt, velocity.right * dt));
            poseTracker.update(totalDisplacement.left, totalDisplacement.right, simAccel.left * dt, simAccel.right * dt, time);

            System.out.println(voltages);

            lastVelocities = velocity;
        }

        return poseTracker;
    }

}
