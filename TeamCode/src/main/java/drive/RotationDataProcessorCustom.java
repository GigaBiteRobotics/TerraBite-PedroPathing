package drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class RotationDataProcessorCustom {
	public ArrayList<Double> dataPoints = new ArrayList<>();
	private static final double outlierThreshold = 1.5;

	public void addDataPoint(double dataPoint) {
		dataPoints.add(dataPoint);
	}
	public void clearDataPoints() {
		dataPoints.clear();
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
	public double getAverage() {
		double sum = 0;
		for (double value : dataPoints) {
			sum += value;
		}
		return sum / dataPoints.size();
	}
	public double getMode() {
		if (dataPoints == null || dataPoints.isEmpty()) {
			return Double.NaN; // Use NaN to indicate an empty dataset.
		}

		// Count the frequency of each number in the list.
		HashMap<Double, Integer> frequencyMap = new HashMap<>();
		for (double num : dataPoints) { // Unboxing happens here
			frequencyMap.put(num, frequencyMap.getOrDefault(num, 0) + 1);
		}

		// Find the key with the highest frequency.
		double mode = Double.NaN;
		int maxCount = 0;
		for (Map.Entry<Double, Integer> entry : frequencyMap.entrySet()) {
			if (entry.getValue() > maxCount) {
				maxCount = entry.getValue();
				mode = entry.getKey();
			}
		}
		return mode;
	}
	public double calculateStandardDeviation(ArrayList<Double> dataPoints, double mean) {
		double sum = 0;
		for (double value : dataPoints) {
			sum += Math.pow(value - mean, 2);
		}

		return Math.sqrt(sum / dataPoints.size());
	}
	public void removeOldDataPoints(int maxDataPoints) {
		while (dataPoints.size() > maxDataPoints) {
			dataPoints.remove(0);
		}
	}
}
