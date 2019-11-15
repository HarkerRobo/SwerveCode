package frc.robot.util;

/**
 * Utility class for doing various vector calculations.
 * 
 * @author Shahzeb Lakhani
 * @author Anirudh Kotamraju
 * @author Chirag Kaushik
 * @author Aimee Wang
 * 
 * @since 11/7/19
 */
public class Vector {
    private double x;
    private double y;
    
    /**
     * Creates a Vector with <0,0>
     */
    public Vector() {
        this(0, 0);
    }

    /**
     * Creates a Vector with <x,y>
     */
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
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

    /**
     * Returns the angle of the vector with respect to the positive x axis in degrees. 
     * Returns a vaue within the range of [0,360)
     */
    public double getAngle() {
        double angle = Math.atan2(y, x) * 180 / Math.PI; //Angle in degrees, range: (-180,180]
        return (angle + 360) % 360; //Convert to desired range
    }

    /**
     * Gets the magnitude of the vector.
     */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns this vector after scaling it by a certain factor
     * 
     * @param scale The value to multiply each of the vector's components by
     */
    public Vector scale(double scale) {
        x *= scale;
        y *= scale;
        return this;
    }

    /**
     * Adds two vectors and returns the sum in a new Vector.
     */
    public static Vector add(Vector first, Vector second) {
        return new Vector(first.getX() + second.getX(), first.getY() + second.getY());
    }

    /**
     * Adds this Vector to another Vector and returns the sum in a new Vector.
     */
    public Vector add(Vector other) {
        return add(this, other);
    }

    /**
     * Rotates this vector about the origin by an angle (in degrees),
     * changing its x and y pos
     * 
     * Positive angle means a counter-clockwise rotation
     */
    public void rotate(double degrees) {
        double radians = degrees * Math.PI / 180;
        double oldX = x;
        double oldY = y;

        x = oldX * Math.cos(radians) - oldY * Math.sin(radians);
        y = oldX * Math.sin(radians) + oldY * Math.cos(radians);
    }
}