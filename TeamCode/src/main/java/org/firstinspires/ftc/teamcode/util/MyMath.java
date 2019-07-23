package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * This class has useful math functions not included in the basic Math class.
 * Using these methods makes code more readable, and putting the methods in a separate class
 * prevents duplication.
 */
public class MyMath {

    public static double median(ArrayList<Double> values)
    {
        Double[] copy = values.toArray(new Double[values.size()]);
        Arrays.sort(copy);

        int size = values.size();
        int half = size / 2;

        if(size % 2 == 1) //if the size is odd
            return copy[half]; //return the middle value

        return (copy[half-1] + copy[half]) / 2;

    }

    public static double ave(ArrayList<Double> values)
    {
        double sum = 0;
        for (double v:values) {
            sum += v;
        }
        return sum / values.size();
    }

    public static double dotProduct(double[] distances, double[] coefficients)
    {
        double sum = 0;
        for(int i = 0; i < distances.length; i++)
        {
            sum += distances[i] * coefficients[i];
        }
        return sum;
    }


    /**
     * @param target angle in radians
     * @param current angle in radians
     * @return the smallest signed difference between the two angles
     */
    public static double loopAngle(Number target, Number current)
    {
        double d = target.doubleValue() - current.doubleValue();
        while(d < -Math.PI) d += 2 * Math.PI; //keeps it between -pi and pi
        while(d >  Math.PI) d -= 2 * Math.PI;
        return d;
    }

    /**
     * This method returns an equivalent angle to the average, but it does not guarantee
     * that angle will be inside a range. This does not affect trig functions, which are periodic.
     * @param angles: an ArrayList of angles in radians.
     * @return the average of several angles
     */
    public static double loopAve(ArrayList<? extends Number> angles)
    {
        double thetaAve = 0;
        for (int n = 0; n < angles.size(); n++) {
            thetaAve += MyMath.loopAngle(angles.get(n), thetaAve) / (n+1);
        }
        return thetaAve;
    }

    /**
     * Finds the largest number (farthest from negative infinity) out of the given doubles
     * @param numbers: doubles separated by commas
     * @return the largest number
     */
    public static double max(double... numbers)
    {
        double max = Double.NEGATIVE_INFINITY;
        for (double n: numbers) {
            if(n > max)
                max = n;
        }
        return max;
    }

    /**
     * Finds the smallest number (farthest from positive infinity) out of the given doubles
     * @param numbers: doubles separated by commas
     * @return the smallest number
     */
    public static double min(double... numbers)
    {
        double min = Double.POSITIVE_INFINITY;
        for (double n: numbers) {
            if(n < min)
                min = n;
        }
        return min;
    }

    public static double max(ArrayList<? extends Number> numbers)
    {
        Number max = Double.NEGATIVE_INFINITY;
        for (Number n: numbers) {
            if(n.doubleValue() > max.doubleValue())
                max = n;
        }
        return max.doubleValue();
    }

    public static double absoluteMax(double... numbers)
    {
        double max = 0;
        for (double n: numbers)
            if(Math.abs(n) > max)
                max = Math.abs(n);
        return max;
    }
    public static double absoluteMax(ArrayList<? extends Number> numbers)
    {
        double max = 0;
        for (Number n: numbers)
            if(Math.abs(n.doubleValue()) > max)
                max = Math.abs(n.doubleValue());
        return max;
    }

    public static double farthestFromZero(double... numbers)
    {
        double max = 0;
        for (double n: numbers)
            if(Math.abs(n) > Math.abs(max))
                max = n;
        return max;
    }

    public static double closestToZero(double... numbers)
    {
        double min = 0;
        if(numbers.length > 0)
            min = numbers[0];

        for (double n: numbers)
            if(Math.abs(n) < Math.abs(min))
                min = n;
        return min;
    }

    public static double limitMagnitude(double value, double limit)
    {
        return closestToZero(value, limit * Math.signum(value));
    }

    public static void trimFromFront(ArrayList<?> list, int size)
    {
        while(list.size() > size && list.size() > 0) {
            list.remove(0);
        }
    }

    public static void fill(ArrayList<Double> doubles, Number n)
    {
        for (int i = 0; i < doubles.size(); i++) {
            doubles.set(i, n.doubleValue());
        }
    }

    public static double radians(double degrees)
    {
        return degrees * Math.PI / 180;
    }

    public static double degrees(double radians)
    {
        return radians * 180 / Math.PI;
    }
}
