package frc.robot.temp;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DynamicTimeWarper {

  public static double calculateDTW(
      List<DataPoint> dataSet1,
      List<DataPoint> dataSet2) {
    int m = dataSet1.size();
    int n = dataSet2.size();

    // Create a distance matrix
    double[][] distanceMatrix = new double[m][n];
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        distanceMatrix[i][j] = Math.abs(dataSet1.get(i).getValue() - dataSet2.get(j).getValue());
      }
    }

    // Initialize the cost matrix
    double[][] costMatrix = new double[m][n];
    for (int i = 0; i < m; i++) {
      Arrays.fill(costMatrix[i], Double.POSITIVE_INFINITY);
    }
    costMatrix[0][0] = distanceMatrix[0][0];

    // Calculate accumulated cost matrix
    for (int i = 1; i < m; i++) {
      for (int j = 1; j < n; j++) {
        double cost = distanceMatrix[i][j];
        costMatrix[i][j] = cost
            + Math.min(costMatrix[i - 1][j], Math.min(costMatrix[i][j - 1], costMatrix[i - 1][j - 1]));
      }
    }

    // Return the DTW distance
    return costMatrix[m - 1][n - 1];
  }

  public static List<DataPoint> injectPointsThroughInterpolation(List<DataPoint> dataSet, Duration interval) {
    List<DataPoint> newDataPoints = new ArrayList<>();
    Instant startTime = dataSet.get(0).getTimestamp();
    Instant endTime = dataSet.get(dataSet.size() - 1).getTimestamp();

    // Iterate over time at regular intervals
    for (Instant time = startTime; time.isBefore(endTime); time = time.plus(interval)) {
      double value = interpolateValue(dataSet, time);
      newDataPoints.add(new DataPoint(time, value));
    }

    return newDataPoints;
  }

  // Method to interpolate the value of a data set at a specific time
  private static double interpolateValue(List<DataPoint> dataSet, Instant time) {
    // Find the two nearest data points to the specified time
    DataPoint prevPoint = null;
    DataPoint nextPoint = null;
    for (DataPoint dataPoint : dataSet) {
      if (dataPoint.getTimestamp().compareTo(time) <= 0) {
        prevPoint = dataPoint;
      } else {
        nextPoint = dataPoint;
        break;
      }
    }

    // If there are no previous or next points, return 0
    if (prevPoint == null || nextPoint == null) {
      return 0.0;
    }

    // Interpolate the value between the two nearest data points
    double timeDiffPrev = Duration.between(prevPoint.getTimestamp(), time).toMillis();
    double timeDiffNext = Duration.between(time, nextPoint.getTimestamp()).toMillis();
    double totalDiff = Duration.between(prevPoint.getTimestamp(), nextPoint.getTimestamp()).toMillis();

    // Avoid division by zero
    if (totalDiff == 0) {
      return 0.0;
    }

    double weightPrev = timeDiffNext / totalDiff;
    double weightNext = timeDiffPrev / totalDiff;

    return prevPoint.getValue() * weightPrev + nextPoint.getValue() * weightNext;
  }

}
