package main;

import control.FeedforwardController;
import control.PolarController;
import csv.ReflectingCSVWriter;
import csv.SimplePose;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import javafx.application.Application;
import javafx.stage.Stage;
import math.geometry.Pose;
import math.geometry.RigidTransform2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;
import physics.DCMotorModel;
import physics.DifferentialDrive;

import java.io.File;

public class Main extends Application {

    public static final double TIMESTEP = 1.0 / 100.0;

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        DCMotorModel motor = new DCMotorModel(1 / 0.135, 1 / 0.012, 1.055);
        DifferentialDrive kinematics = new DifferentialDrive(25.5, /*1.6*/1.0, 5.875, 1024,
                                                            60.0, 10.0, 12.0,
                                                            motor, motor);

//        simulatePolar(4.0, 6.0, 0.9, physics);
        simulateFeedforward(0.65, 0.175, kinematics);

          System.out.println("Done");
          System.exit(0);
    }

    public void simulatePolar(double k1, double k2, double k3, DifferentialDrive kinematics) {
        RigidTransform2d initialPosition = new RigidTransform2d(new Translation2d(50.0, -50.0), Rotation2d.fromDegrees(0270.0));

        PolarController controller = new PolarController(k1, k2, k3, kinematics);
        controller.setPath(Pathfinder.readFromCSV(new File("trajectory.csv")));

        SimulationHarness simulator = new SimulationHarness(controller, new PoseTracker(kinematics), kinematics);
        PoseTracker poseTracker = simulator.simulate(TIMESTEP, 300.0, initialPosition);

        writeToCsv(poseTracker);
    }

    public void simulateFeedforward(double squiggly, double b, DifferentialDrive kinematics) {
        RigidTransform2d initialPosition = new RigidTransform2d(new Translation2d(50.0, -50.0), Rotation2d.fromDegrees(0270.0));

        FeedforwardController controller = new FeedforwardController(squiggly, b, kinematics);
        controller.setTrajectory(generateTrajectory());

        SimulationHarness simulator = new SimulationHarness(controller, new PoseTracker(kinematics), kinematics
        );
        PoseTracker poseTracker = simulator.simulate(TIMESTEP, 300.0, initialPosition);

        writeToCsv(poseTracker);
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

    public double getNoise() {
        return (Math.random() * 400) - 200;
    }

}
