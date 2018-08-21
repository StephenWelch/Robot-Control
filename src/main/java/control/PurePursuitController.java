package control;

import adapter.PathSegment;
import physics.Movement;
import math.geometry.Pose;

import java.util.List;

public class PurePursuitController implements IController {

    private final double lookahead;

    private List<PathSegment> path;

    public PurePursuitController(double lookahead) {
        this.lookahead = lookahead;
    }


    @Override
    public Movement update(Pose pose, double dt) {
        return null;
    }

    @Override
    public void reset() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
