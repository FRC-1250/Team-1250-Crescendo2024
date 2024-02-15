package frc.robot.util;

public class RobotHelper {
    /*
     * @param currentValue
     * 
     * @param targetValue
     * 
     * @param acceptancePercent the percentage away from the target in the positive
     * or negative direction is acceptable to provide a true response
     * 
     * The upperBound is calculated as: targetValue + (targetValue *
     * acceptancePercent)
     * The lowerBound is calculated as: lowerValue = targetValue - (targetValue *
     * acceptancePercent)
     */
    @Deprecated
    public static boolean isWithinRangeOfTarget(double currentValue, double targetValue, double acceptancePercent) {
        var upperValue = targetValue + (targetValue * acceptancePercent);
        var lowerValue = targetValue - (targetValue * acceptancePercent);
        return currentValue >= lowerValue && currentValue <= upperValue;
    }
}
