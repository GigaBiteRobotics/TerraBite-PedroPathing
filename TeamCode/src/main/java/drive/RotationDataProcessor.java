package drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class RotationDataProcessor {
	public ArrayList<Double> dataPoints = new ArrayList<>();
	public ArrayList<Double> dataPointAges = new ArrayList<>();
	private static final double outlierThreshold = 1.5;
	public int counter = 0;
	public RotationDataProcessor(double maxAge) {
	}

	public void addDataPoint(double dataPoint, double datapointAge) {
		dataPoints.add(dataPoint);
		dataPointAges.add(datapointAge);
		counter ++;
	}
	public void clearDataPoints() {
		dataPoints.clear();
		dataPointAges.clear();
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
			return Double.NaN;
		}

		HashMap<Double, Integer> frequencyMap = new HashMap<>();
		for (double num : dataPoints) {
			frequencyMap.merge(num, 1, Integer::sum);
		}

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
	public void removeOldDataPoints(int maxDataPoints) {
		while (dataPoints.size() > maxDataPoints) {
			dataPoints.remove(0);
			dataPointAges.remove(0);
		}
		for (int i = 0; i < dataPoints.size(); i++) {
			if (dataPointAges.get(i) > 1000 + dataPointAges.get(dataPointAges.size() - 1)) {
				dataPoints.remove(i);
				dataPointAges.remove(i);
				i--;
			}
		}
	}
}
