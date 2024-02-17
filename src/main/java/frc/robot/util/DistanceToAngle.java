public class DistanceToAngle {
    //Name: distAngle
    //Parameters: inDistance
    //Returns: Angle for shooter
    //Uses 2 lists of related distnaces(x) and Angles(y) to create a peicwise function to find a angle for a distance
    public static double distAngle (double inDistance){
        //Distance (pre programed)
        double[] distances = {0,5,10};
        //Angles (pre programed)
        double[] angels = {50, 40, 10};
        //INDEX MUST MATCH
        
        //return
        double outAngle = 0;

        //for creating function
        double xlower = 0, xupper = 0, ylower= 0, yupper = 0, slope = 0;

        //Travers list to find upper and lower values
        for (int i = 0; i<distances.length; i++) {
            if (distances[i] > inDistance){
                
                xlower = distances[i-1];
                xupper = distances[i];
                ylower = angels[i-1];
                yupper = angels[i];
                break;
            }
            if(distances[i] == inDistance){
                return angels[i];
            }
        }
        //create linare function with values
        slope = (yupper-ylower)/(xupper-xlower);
        outAngle = (slope*inDistance) - (slope*xlower) + ylower;
        System.out.println("Slope, xu, xl, yu, yl");
        System.out.println(slope);
        System.out.println(xupper);
        System.out.println(xlower);
        System.out.println(yupper);
        System.out.println(ylower);
        System.out.println("Return");
        System.out.println(outAngle);
        return outAngle;
    }

}
