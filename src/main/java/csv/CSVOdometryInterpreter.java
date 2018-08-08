package csv;

import com.team254.lib.util.ReflectingCSVWriter;
import javafx.stage.Stage;
import kinematics.DifferentialDriveKinematics;
import kinematics.Pose;
import kinematics.PoseTracker;

import java.util.List;

public class CSVOdometryInterpreter {

    private CSVReader csvReader;
    private ReflectingCSVWriter<SimplePose> csvWriter;
    private DifferentialDriveKinematics kinematics;
    private PoseTracker stateEstimator;
    private Stage stage;

    public CSVOdometryInterpreter(String logPath, String outputPath, DifferentialDriveKinematics kinematics, Stage stage) {
        this.csvReader = new CSVReader(logPath);
        this.csvWriter = new ReflectingCSVWriter<>(outputPath, SimplePose.class);
        this.kinematics = kinematics;
        this.stateEstimator = new PoseTracker(kinematics);
        this.stage = stage;
    }

    public PoseTracker read() {
        for(List<Double> row : csvReader.getAsDoubles()) {
            if (row.isEmpty() || row.get(4) == -1 || row.get(5) == -1) {
                System.out.println("Skipping line: invalid data");
                continue;
            }
            double leftAbsolutePosition = kinematics.ticksToDistance(row.get(4).intValue());
            double rightAbsolutePosition = kinematics.ticksToDistance(row.get(5).intValue());
            double leftVelocity = row.get(14) * 12; // FPS to IPS
            double rightVelocity = row.get(15) * 12;
            double timestamp = row.get(32);
            stateEstimator.update(leftAbsolutePosition, rightAbsolutePosition, leftVelocity, rightVelocity, timestamp);
        }
        return stateEstimator;
    }

    public void write(double interval) {
        double maxTime = stateEstimator.getLatestTime();
        for (double timestamp = 0L; timestamp <= maxTime; timestamp += interval) {
            Pose latest = stateEstimator.get(timestamp);

            SimplePose pose = new SimplePose(latest.position.getTranslation().x(), latest.position.getTranslation().y(), latest.position.getRotation().getDegrees(),
                    latest.velocity.getTranslation().x(), latest.velocity.getTranslation().y(), latest.velocity.getRotation().getDegrees(),
                    timestamp);
            csvWriter.add(pose);
        }
        csvWriter.write();
    }

}
