package physics;

import math.Util;

public class DCMotorModel {

    public final double SPEED_PER_VOLT, ACCEL_PER_VOLT, FRICTION_VOLTAGE;

    public DCMotorModel(double SPEED_PER_VOLT, double ACCEL_PER_VOLT, double FRICTION_VOLTAGE) {
        this.SPEED_PER_VOLT = SPEED_PER_VOLT;
        this.ACCEL_PER_VOLT = ACCEL_PER_VOLT;
        this.FRICTION_VOLTAGE = FRICTION_VOLTAGE;
    }

    public double getVoltage(double velocity, double acceleration) {

        double frictionVoltage = 0.0;

        if(!Util.epsilonEquals(velocity, 0)) {
            frictionVoltage = Math.signum(velocity) * FRICTION_VOLTAGE;
        } else if(!Util.epsilonEquals(acceleration, 0)) {
            frictionVoltage = Math.signum(acceleration) * FRICTION_VOLTAGE;
        }

        return (velocity / SPEED_PER_VOLT) + (acceleration / ACCEL_PER_VOLT) - frictionVoltage;

    }

    public double getAcceleration(double voltage, double velocity) {
        double effectiveVoltage = voltage;
        double velocityVoltage = voltage / SPEED_PER_VOLT;

        if(!Util.epsilonEquals(velocity, 0)) {
            effectiveVoltage = effectiveVoltage - (Math.signum(velocity) * FRICTION_VOLTAGE);
        } else if(voltage > Util.EPSILON) {
            effectiveVoltage = Math.max(0.0, effectiveVoltage - FRICTION_VOLTAGE);
        } else if(voltage < Util.EPSILON) {
            effectiveVoltage = Math.min(0.0, effectiveVoltage + FRICTION_VOLTAGE);
        }

        effectiveVoltage = effectiveVoltage - velocity;

        return ACCEL_PER_VOLT * effectiveVoltage;
    }

}
