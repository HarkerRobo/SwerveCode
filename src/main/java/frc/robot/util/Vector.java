package frc.robot.util;

/**
 * Utility class for doing various vector calculations.
 * 
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Aimee Wang
 * 
 * @since November 7, 2019
 */
public class Vector {
    private double x;
    private double y;
    
    public Vector() {
        this(0, 0);
    }

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Adds two vectors and returns the sum.
     * 
     * TODO: scaling down using omega factor
     */
    public static Vector add(Vector first, Vector second) {
        Vector newVector = new Vector(first.getX() + second.getX(), first.getY() + second.getY());
        // double scale = Math.max(first.getMagnitude(),second.getMagnitude())
        return newVector;
    }

    /**
     * Adds the current vector to another vector and returns the sum.
     */
    public Vector add(Vector other) {
        return add(this, other);
    }
    
    /**
     * Gets the magnitude of the vector.
     */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns the angle of the vector with respect to the positive x axis in degrees. 
     * Returns a vaue within the range of [0,360)
     */
    public double getAngle() {
        return ((Math.atan2(y, x) * 180 / Math.PI) + 180) % 360;
    }

    /**
     * Gets the X value.
     */
    public double getX() {
        return x;
    }
    
    /**
     * Gets the Y value.
     */
    public double getY() {
        return y;
    }

    public Vector scale(double scale) {
        x *= scale;
        y *= scale;
        return this;
    }

}