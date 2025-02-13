package drive;

import java.util.ArrayList;

public class RotationDataProcessorCustom {
	private ArrayList<Double> dataPoints = new ArrayList<>();
	private static final double outlierThreshold = 1.5;

	public void addDataPoint(double dataPoint) {
		dataPoints.add(dataPoint);
	}
	public void clearDataPoints() {
		dataPoints.clear();
	}
	public double getAverage() {
		double sum = 0;
		for (double point : dataPoints) {
			sum += point;
		}
		return sum / dataPoints.size();
	}
	public void removeOutliers() {
		if (dataPoints.size() < 3) return;
		double mean = getAverage();
		double standardDeviation = calculateStandardDeviation(dataPoints, mean);
		ArrayList<Double> filteredData = new ArrayList<>();
		for (double value : dataPoints) {
			if (Math.abs(value - mean) <= outlierThreshold * standardDeviation) {
				filteredData.add(value);
			}
		}
		dataPoints = filteredData;
	}
	private double calculateStandardDeviation(ArrayList<Double> data, double mean) {
		double sum = 0;
		for (double value : data) {
			sum += Math.pow(value - mean, 2);
		}
		return Math.sqrt(sum / data.size());
	}
}
