package dynamics;

public class Feedforward {

    public final double velocityFeedforward, velocityIntercept, accelFeedforward, accelIntercept;

    public Feedforward(double velocityFeedforward, double velocityIntercept, double accelFeedforward, double accelIntercept) {
        this.velocityFeedforward = velocityFeedforward;
        this.velocityIntercept = velocityIntercept;
        this.accelFeedforward = accelFeedforward;
        this.accelIntercept = accelIntercept;
    }

    public double getOutput(double velocity, double accel) {
        double velocityOutput = (velocity * velocityFeedforward) + velocityIntercept;
        double accelOutput = (accel * accelFeedforward) + accelIntercept;
        return velocityOutput + accelOutput;
    }

}
