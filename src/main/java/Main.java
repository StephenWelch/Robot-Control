import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;
import control.FeedforwardController;
import control.PolarController;
import csv.SimplePose;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import javafx.application.Application;
import javafx.stage.Stage;
import kinematics.DifferentialDriveKinematics;
import kinematics.Movement;
import kinematics.Pose;
import kinematics.PoseTracker;

import java.io.File;

public class Main extends Application {

    public static final double TIMESTEP = 1.0 / 100.0;

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(25.5, 1.6, 5.875, 1024);

//        simulatePolar(4.0, 6.0, 0.9, kinematics);
        simulateFeedforward(0.65, 0.175, kinematics);

          System.out.println("Done");
          System.exit(0);
    }

    public void simulatePolar(double k1, double k2, double k3, DifferentialDriveKinematics kinematics) {
        PoseTracker poseTracker = new PoseTracker(kinematics);
        poseTracker.reset(new RigidTransform2d(new Translation2d(50.0, -50.0), Rotation2d.fromDegrees(0.0)), 0.0);

        PolarController controller = new PolarController(k1, k2, k3, kinematics);
        controller.setPath(Pathfinder.readFromCSV(new File("trajectory.csv")));

        Movement totalDisplacement = new Movement(0.0, 0.0);

        for(double time = 0.0; !controller.isFinished(); time += TIMESTEP) {
            Movement velocity = controller.update(poseTracker.getLatest(), 0.0);
            velocity = clampVelocities(velocity, 120.0);
            totalDisplacement = totalDisplacement.add(new Movement(velocity.left * TIMESTEP, velocity.right * TIMESTEP));
            poseTracker.update(totalDisplacement.left, totalDisplacement.right, velocity.left, velocity.right, time);

        }
        writeToCsv(poseTracker);
    }

    public Translation2d simulateFeedforward(double squiggly, double b, DifferentialDriveKinematics kinematics) {
        PoseTracker poseTracker = new PoseTracker(kinematics);
        poseTracker.reset(new RigidTransform2d(new Translation2d(50.0, -50.0), Rotation2d.fromDegrees(0270.0)), 0.0);

        FeedforwardController controller = new FeedforwardController(squiggly, b, kinematics);
        controller.setTrajectory(generateTrajectory());

        Movement totalDisplacement = new Movement(0.0, 0.0);

        Translation2d trackErrorAccum = new Translation2d();
        int iterations = 0;

        for(double time = 0.0; !controller.isFinished(); time += TIMESTEP) {

            Movement velocity = controller.update(poseTracker.getLatest(), 0.0);
            velocity = clampVelocities(velocity, 120.0);
            //velocity = new Movement(velocity.left + 50, velocity.right + 50);
            totalDisplacement = totalDisplacement.add(new Movement(velocity.left * TIMESTEP, velocity.right * TIMESTEP));
            poseTracker.update(totalDisplacement.left, totalDisplacement.right, velocity.left, velocity.right, time);

            // Update stats
            trackErrorAccum = trackErrorAccum.translateBy(controller.getTrackError());
            iterations++;
        }
        writeToCsv(poseTracker);
        return new Translation2d(trackErrorAccum.x() / iterations, trackErrorAccum.y() / iterations);
    }

    public Trajectory generateTrajectory() {
        Waypoint[] waypoints = new Waypoint[]{
                new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
                new Waypoint(300, 300, Pathfinder.d2r(0.0))
        };

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, TIMESTEP, 120.0, 30.0, 720.0);
        Trajectory trajectory = Pathfinder.generate(waypoints, config);
        Pathfinder.writeToCSV(new File("trajectory.csv"), trajectory);
        return trajectory;
    }

    public void writeToCsv(PoseTracker poseTracker) {
        ReflectingCSVWriter<SimplePose> writer = new ReflectingCSVWriter<SimplePose>("tracking.csv", SimplePose.class);
        for(double time = 0.0; time < 300.0; time += TIMESTEP) {
            Pose latest = poseTracker.get(time);
            SimplePose pose = new SimplePose(latest.position.getTranslation().x(), latest.position.getTranslation().y(), latest.position.getRotation().getDegrees(),
                    latest.velocity.getTranslation().x(), latest.velocity.getTranslation().y(), latest.velocity.getRotation().getDegrees(),
                    time);
            writer.add(pose);
            writer.write();
        }
    }

    public Movement clampVelocities(Movement movement, double magnitude) {
        double left = movement.left;
        double right = movement.right;
        if(Math.abs(movement.left) > magnitude) {
            left = Math.signum(movement.left) * magnitude;
        }
        if(Math.abs(movement.right) > magnitude) {
            right = Math.signum(movement.right) * magnitude;
        }
        return new Movement(left, right);
    }

    public Movement addNoise(Movement movement) {
        return new Movement(movement.left + getNoise(), movement.right + getNoise());
    }

    public double getNoise() {
        return (Math.random() * 400) - 200;
    }

}
