package adapter;

import jaci.pathfinder.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class PathAdapter {

    private final Trajectory trajectory;
    private final List<PathSegment> segments = new ArrayList<>();

    public PathAdapter(Trajectory trajectory) {
        this.trajectory = trajectory;
        for(Trajectory.Segment s : trajectory.segments) {
            segments.add(new PathSegmentAdapter(s));
        }
    }

    public List<PathSegment> getSegments() {
        return segments;
    }

}
