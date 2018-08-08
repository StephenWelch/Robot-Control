package adapter;

import jaci.pathfinder.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryAdapter {

    private final Trajectory trajectory;
    private final List<TrajectorySegment> segments;

    public TrajectoryAdapter(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.segments = new ArrayList<>();
        for(Trajectory.Segment s : trajectory.segments) {
            segments.add(new TrajectorySegmentAdapter(s));
        }
    }

    public List<TrajectorySegment> getSegments() {
        return segments;
    }

}
