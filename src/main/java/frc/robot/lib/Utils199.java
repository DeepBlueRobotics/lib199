package frc.robot.lib;

public final class Utils199 {

    public static double average(double[] arr) {
        double sum = 0;
        for (double x : arr) sum += x;
        return sum / arr.length;
    }

    private Utils199() {}

}
