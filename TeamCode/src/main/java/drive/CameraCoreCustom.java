package drive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.openftc.apriltag.*;

public class CameraCoreCustom {
	Limelight3A limelight;
	double[] LLColorPoints;
	LLResult result;
	List<LLResultTypes.ColorResult> colorResults;
	AprilTagDetection aprilTagDetection = new AprilTagDetection();
	LimelightDataFetcher llDataFetcher = new LimelightDataFetcher();
	HashMap<Object, Object> jsonDataMap = new HashMap<>();
	Object retroData;

	public void limelightCoreCustomInit(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(100);
		limelight.start();
	}

	public void setPipeline(int pl) {
		if (pl < 10 && pl > 0) {
			limelight.pipelineSwitch(pl);
			limelight.setPollRateHz(100);
			limelight.start();
		}
	}

	/*
	public List<LLResultTypes.ColorResult> getColor(int color) {
		if (color == 0) limelight.pipelineSwitch(0);
		else if (color == 1) limelight.pipelineSwitch(1);
		else if (color ==2) limelight.pipelineSwitch(2);
		result = limelight.getLatestResult();
		colorResults = result.getColorResults();
		return colorResults;
	}
	public boolean getHand(int human) {
		if (human == 0) limelight.pipelineSwitch(3);
		else limelight.pipelineSwitch(4);
		result = limelight.getLatestResult();
		colorResults = result.getColorResults();
		if (result != null && result.isValid()) return result.getTa() > 1;
		else return false;
	}
	*/
	public double[][] getColorPoints() throws Exception {
		return llDataFetcher.getPoints();
	}

	public double getColorAngle() throws Exception {
		double[][] pts = llDataFetcher.getPoints();
		double[] postPts = findLongestAndSecondLongestDistances(pts);
		return postPts[2];
	}

	/**
	 * Computes the angle of a rectangle (defined by 4 points) by comparing its two diagonals.
	 * The longer diagonal is assumed to represent the rectangle's long axis.
	 *
	 * @param points a 4x2 array where each sub-array contains the x and y coordinates of a corner.
	 * @return the angle (in degrees) of the rectangle's long axis relative to the x-axis.
	 */
	public static double calculateRectangleAngle(double[][] points) {
		int lengthPoints = (points.length);
		String strPoints = Integer.toString(lengthPoints);
		if (points.length != 4) {
			return 0;
		}

		// Calculate the two diagonals
		double[] diagonal1 = {points[0][0] - points[2][0], points[0][1] - points[2][1]};
		double[] diagonal2 = {points[1][0] - points[3][0], points[1][1] - points[3][1]};

		// Choose the longer diagonal
		double length1 = Math.sqrt(diagonal1[0] * diagonal1[0] + diagonal1[1] * diagonal1[1]);
		double length2 = Math.sqrt(diagonal2[0] * diagonal2[0] + diagonal2[1] * diagonal2[1]);
		double[] longAxis = (length1 >= length2) ? diagonal1 : diagonal2;

		// Calculate the angle of the long axis relative to the x-axis
		double angleRadians = Math.atan2(longAxis[1], longAxis[0]);
		double angleDegrees = Math.toDegrees(angleRadians);

		return angleDegrees;
	}

	public static double[][] getFurthestPoints(double[][] points) {
		if (points.length < 4) {
			throw new IllegalArgumentException("At least 4 points are required.");
		}

		// Find the two points with the maximum distance
		double maxDistance = 0;
		int[] furthestPair = new int[2];
		for (int i = 0; i < points.length; i++) {
			for (int j = i + 1; j < points.length; j++) {
				double distance = Math.sqrt(
						Math.pow(points[j][0] - points[i][0], 2) +
								Math.pow(points[j][1] - points[i][1], 2)
				);
				if (distance > maxDistance) {
					maxDistance = distance;
					furthestPair[0] = i;
					furthestPair[1] = j;
				}
			}
		}

		// Collect the four points furthest apart from each other
		boolean[] included = new boolean[points.length];
		included[furthestPair[0]] = true;
		included[furthestPair[1]] = true;

		double[][] furthestPoints = new double[4][2];
		furthestPoints[0] = points[furthestPair[0]];
		furthestPoints[1] = points[furthestPair[1]];

		int count = 2;
		while (count < 4) {
			double maxMinDistance = 0;
			int nextPointIndex = -1;

			for (int i = 0; i < points.length; i++) {
				if (included[i]) continue;

				double minDistance = Double.MAX_VALUE;
				for (int j = 0; j < count; j++) {
					double distance = Math.sqrt(
							Math.pow(points[i][0] - furthestPoints[j][0], 2) +
									Math.pow(points[i][1] - furthestPoints[j][1], 2)
					);
					minDistance = Math.min(minDistance, distance);
				}

				if (minDistance > maxMinDistance) {
					maxMinDistance = minDistance;
					nextPointIndex = i;
				}
			}

			included[nextPointIndex] = true;
			furthestPoints[count] = points[nextPointIndex];
			count++;
		}

		return furthestPoints;
	}

	public double[] findLongestAndSecondLongestDistances(double[][] points) {
		if (points.length < 4) {
			throw new IllegalArgumentException("At least 4 points are required.");
		}

		// Find the two points with the longest distance.
		double maxDistance = 0;
		int p1Index = -1, p2Index = -1;

		for (int i = 0; i < points.length; i++) {
			for (int j = i + 1; j < points.length; j++) {
				double distance = calculateDistance(points[i], points[j]);
				if (distance > maxDistance) {
					maxDistance = distance;
					p1Index = i;
					p2Index = j;
				}
			}
		}

		// Exclude the points used in the first calculation.
		List<double[]> remainingPoints = new ArrayList<>();
		for (int i = 0; i < points.length; i++) {
			if (i != p1Index && i != p2Index) {
				remainingPoints.add(points[i]);
			}
		}

		// Find the two points with the second-longest distance.
		double secondMaxDistance = 0;
		int p3Index = -1, p4Index = -1;

		for (int i = 0; i < remainingPoints.size(); i++) {
			for (int j = i + 1; j < remainingPoints.size(); j++) {
				double distance = calculateDistance(remainingPoints.get(i), remainingPoints.get(j));
				if (distance > secondMaxDistance) {
					secondMaxDistance = distance;
					p3Index = i;
					p4Index = j;
				}
			}
		}

		// Calculate angles from vertical for the two lines.
		double angle1 = normalizeAngle(calculateAngleFromVertical(points[p1Index], points[p2Index]));
		double angle2 = normalizeAngle(calculateAngleFromVertical(remainingPoints.get(p3Index), remainingPoints.get(p4Index)));

		// Compute the average angle.
		double averageAngle = (angle1 + angle2) / 2;

		return new double[]{maxDistance, secondMaxDistance, averageAngle};
	}

	private static double calculateDistance(double[] p1, double[] p2) {
		double dx = p2[0] - p1[0];
		double dy = p2[1] - p1[1];
		return Math.sqrt(dx * dx + dy * dy);
	}

	private static double calculateAngleFromVertical(double[] p1, double[] p2) {
		double dx = p2[0] - p1[0];
		double dy = p2[1] - p1[1];
		double angle = Math.toDegrees(Math.atan2(dx, dy)); // Angle relative to the vertical (y-axis)
		return angle < 0 ? angle + 180 : angle;
	}

	private static double normalizeAngle(double angle) {
		return angle % 180; // Ensure angle stays within [0, 180)
	}

	/**
	 * Checks whether a rectangle is in view.
	 * This method attempts to retrieve the color points and returns true only if exactly 4 points are detected.
	 *
	 * @return true if exactly 4 points (defining a rectangle) are detected; false otherwise.
	 */
	public boolean isRectangleInView() {
		try {
			double[][] points = getColorPoints();
			return (points != null && points.length == 4);
		} catch (Exception e) {
			System.out.println("Error in isRectangleInView: " + e.getMessage());
			return false;
		}
	}


	/**
	 * Returns the computed angle of the detected rectangle for external use.
	 * <p>
	 * This method first verifies that a rectangle is in view (i.e. exactly 4 points are detected)
	 * and then calculates the angle using the rectangle's longer diagonal.
	 *
	 * @return the computed angle (in degrees) of the rectangle.
	 * @throws IllegalStateException if no rectangle is detected.
	 * @throws Exception             if an error occurs while retrieving the points.
	 */
	public double getComputedAngle() throws Exception {
		if (!isRectangleInView()) {
			System.out.println("No rectangle detected!");
			return 0;
		}
		double[][] points = getColorPoints();
		if (points == null || points.length < 4) {
			System.out.println("Invalid points received: " + (points == null ? "null" : points.length));
			return 0;
		}
		return calculateRectangleAngle(points);
	}
}
