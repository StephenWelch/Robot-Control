package csv;

public class SimplePose {

    public final double pos_x, pos_y, pos_theta, vel_x, vel_y, vel_theta, timestamp;

    public SimplePose(double pos_x, double pos_y, double pos_theta, double vel_x, double vel_y, double vel_theta, double timestamp) {
        this.pos_x = pos_x;
        this.pos_y = pos_y;
        this.pos_theta = pos_theta;
        this.vel_x = vel_x;
        this.vel_y = vel_y;
        this.vel_theta = vel_theta;
        this.timestamp = timestamp;
    }
}
