package frc.robot.util;

/**
 * vector 2 class cuz i want 1
 * @author florpy
 */
public record Vec2(double x, double y) {

    public double dot(Vec2 other) {
        return x() * other.x() + y() * other.y();
    }

    public double mag() {
        return Math.hypot(x(), y());
    }

    public double toAngle() {
        return RobotMath.vecToDeg(x(), y());
    }

    public Vec2 plus(Vec2 other) {
        return new Vec2(x() + other.x(), y() + other.y());
    }

    public Vec2 minus(Vec2 other) {
        return new Vec2(x() - other.x(), y() - other.y());
    }

    public Vec2 times(double scalar) {
        return new Vec2(x() * scalar, y() * scalar);
    }

    public Vec2 div(double scalar) {
        double factor = 1.0 / scalar;
        return times(factor);
    }

}
