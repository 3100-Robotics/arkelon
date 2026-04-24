package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public interface Utils {

    public enum ShootType {
        Hub, Pass
    }

    public enum Direction {
        Left, Right;
    }

    public static class Acceleration {
        private double last = 0;
        private double current = 0;

        private double lastTime = 0;

        private double acceleration = 0;

        public Acceleration(double tolerance) {
            lastTime = Timer.getFPGATimestamp();
        }

        public void update(double newcurrent) {
            last = current;
            current = newcurrent;
            acceleration = (current - last) / (Timer.getFPGATimestamp() - lastTime);
            lastTime = Timer.getFPGATimestamp();
        }

        public double getAccel() {
            return acceleration;
        }
    }

    public static boolean is_between(double angle, double high, double low) {
        double radianTheta = Math.toRadians(angle);
        double radianA = Math.toRadians(high);
        double radianB = Math.toRadians(low);

        double crossOne = sign(cross_product(radianA, radianTheta));
        double crossTwo = sign(cross_product(radianTheta, radianB));
        double crossThree = sign(cross_product(radianA, radianB));

        return crossOne == crossTwo && crossTwo == crossThree;
    }


    public static double cross_product(double high, double low) {
        var first_x = Math.cos(high);
        var first_y = Math.sin(high);

        var second_x = Math.cos(low);
        var second_y = Math.sin(low);

        return first_x * second_y - first_y * second_x;
    }

    public static double sign(double x) {
        return Math.copySign(1, x);
    }
}
