import com.team254.lib.util.math.RigidTransform2d;
import control.IController;
import kinematics.Movement;
import kinematics.PoseTracker;

public class SimulationHarness {

    private IController controller;
    private PoseTracker poseTracker;

    public SimulationHarness(IController controller, PoseTracker poseTracker) {
        this.controller = controller;
        this.poseTracker = poseTracker;
    }

    public PoseTracker simulate(double dt, double maxTime) {
        return simulate(dt, maxTime, new RigidTransform2d());
    }

    public PoseTracker simulate(double dt, double maxTime, RigidTransform2d initialPosition) {

        poseTracker.reset(initialPosition, 0.0);
        Movement totalDisplacement = new Movement(0.0, 0.0);


        for(double time = 0.0; !controller.isFinished(); time += dt) {
            Movement velocity = controller.update(poseTracker.getLatest(), 0.0);
            totalDisplacement = totalDisplacement.add(new Movement(velocity.left * dt, velocity.right * dt));
            poseTracker.update(totalDisplacement.left, totalDisplacement.right, velocity.left, velocity.right, time);
        }

        return poseTracker;
    }

}
