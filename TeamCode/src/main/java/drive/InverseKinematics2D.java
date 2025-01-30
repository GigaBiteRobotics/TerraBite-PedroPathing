package drive;

/**
 * This class provides a 2D inverse kinematics model for controlling a robotic arm.
 * It calculates the angle and length required for the arm to reach a target (x, y) coordinate,
 * while considering the physical constraints of the arm.
 */
public class InverseKinematics2D {
    // Arm parameters
    public double minAngle; // Minimum angle the arm can rotate to (in radians)
    public double maxAngle; // Maximum angle the arm can rotate to (in radians)
    public double minLength; // Minimum length of the arm (in cm)
    public double maxLength; // Maximum length of the arm (in cm)
    public double maxX; // Maximum allowed x-coordinate (imaginary wall constraint)

    /**
     * Constructor for the InverseKinematics2D class.
     *
     * @param minAngle Minimum angle in radians.
     * @param maxAngle Maximum angle in radians.
     * @param minLength Minimum arm length in cm.
     * @param maxLength Maximum arm length in cm.
     * @param maxX Maximum x-coordinate (used to define an imaginary boundary).
     */
    public InverseKinematics2D(double minAngle, double maxAngle, double minLength, double maxLength, double maxX) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.minLength = minLength;
        this.maxLength = maxLength;
        this.maxX = maxX;
    }

    /**
     * Solves for the arm configuration (angle and length) needed to reach the target point (x, y).
     *
     * @param x Target x-coordinate (horizontal axis) in cm.
     * @param y Target y-coordinate (vertical axis) in cm.
     * @return An array where:
     *         - The first element is the joint angle in radians.
     *         - The second element is the arm length in cm.
     *         Returns null if the target is out of reach or beyond the x-coordinate limit.
     */
    public double[] solve(double x, double y) {
        // Check if the target x-coordinate exceeds the maximum allowed value
        if (x > maxX) {
            return null; // Target is beyond the imaginary wall
        }

        // Calculate the distance (length) and angle required to reach the target
        double desiredLength = Math.sqrt(x * x + y * y); // Length using Pythagoras' theorem
        double desiredAngle = Math.atan2(y, x); // Angle using arctangent

        // Check if the computed length and angle are within valid bounds
        if (desiredLength < minLength || desiredLength > maxLength ||
                desiredAngle < minAngle || desiredAngle > maxAngle) {
            return null; // Target is out of reach
        }

        // Return the valid angle and length
        return new double[] { desiredAngle, desiredLength };
    }

    /**
     * Computes the (x, y) coordinates based on the given angle and length.
     *
     * @param angle The angle in radians.
     * @return An array where:
     *         - The first element is the x-coordinate.
     *         - The second element is the y-coordinate.
     */
    public double computeMaxLength(double angle) {
        // Check if the angle is less than 45 degrees
        angle = angle / 23.2183333;
        if (angle < 45) {
            double radiansAngle = Math.toRadians(angle); // Convert angle to radians
            return 2100 / Math.cos(radiansAngle); // Calculate maximum length based on angle
        } else {
            return maxLength; // Default to maximum length
        }
    }

    /**
     * Example usage of the InverseKinematics2D class.
     */
    public static void main(String[] args) {
        // Initialize arm constraints
        double minAngle = Math.toRadians(-90); // Minimum angle (-90 degrees in radians)
        double maxAngle = Math.toRadians(90);  // Maximum angle (90 degrees in radians)
        double minLength = 10.0; // Minimum arm length (10.0 cm)
        double maxLength = 100.0; // Maximum arm length (100.0 cm)
        double maxX = 100.0; // Maximum x-coordinate (100.0 cm)
        // Create an instance of the InverseKinematics2D class
        InverseKinematics2D arm = new InverseKinematics2D(minAngle, maxAngle, minLength, maxLength, maxX);

        // Define a target point
        double targetX = 50.0; // Target x-coordinate in cm
        double targetY = 50.0; // Target y-coordinate in cm

        // Solve for the arm configuration to reach the target
        double[] result = arm.solve(targetX, targetY);

        // Check if the target is reachable and print the result
        if (result != null) {
            System.out.println("Target reachable.");
            System.out.println("Joint Angle (radians): " + result[0]);
            System.out.println("Arm Length (cm): " + result[1]);
        } else {
            System.out.println("Target out of reach.");
        }
    }
}
