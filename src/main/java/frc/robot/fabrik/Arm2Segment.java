package frc.robot.fabrik;

import java.util.Arrays;

public class Arm2Segment {

    private double[] armLengths;
    private Point[] points = new Point[3];
    private Point goal;

    private double maxReach;

    private final Point armOriginPoint;

    private double errorMargin;

    public Arm2Segment(double[] armLengths, double errorMargin, Point armOriginPoint){
        this.armLengths = armLengths;
        for(int x = 0; x < points.length; x++){
            points[x] = new Point();
        }
        setOptimalCalcStartPos();

        goal = new Point();
        this.armOriginPoint = new Point(armOriginPoint.getX(), armOriginPoint.getY());

        this.errorMargin = errorMargin;

        maxReach = Arrays.stream(armLengths).sum();
    }

    public void setOptimalCalcStartPos(){
        points[1].setCoords(0, armLengths[0]);
        points[2].setCoords(armLengths[1], armLengths[0]);
    }

    public void setGoal(Point goal){
        this.goal.setCoords(goal);
    }

    public boolean outOfMargin(){
        return Math.sqrt(Math.pow((goal.getX()-points[2].getX()),2) + Math.pow((goal.getY()-points[2].getY()),2)) > errorMargin;
    }

    public boolean goalIsReachable(){
        return new Vector(goal, armOriginPoint).getMagnitude() <= maxReach;
    }

    public void fabrik(){

        setOptimalCalcStartPos();

        int i = 0;

        Point[] pointsPrime = new Point[3];

        for(int x = 0; x < pointsPrime.length; x++){
            pointsPrime[x] = new Point();
        }

        if(goalIsReachable()) {
            while (outOfMargin() && i < 100) {

                //FORWARD

                pointsPrime[2].setCoords(goal);
                pointsPrime[1].setCoords(getNextPrime(pointsPrime[2], points[1], armLengths[1]));
                pointsPrime[0].setCoords(getNextPrime(pointsPrime[1], points[0], armLengths[0]));

                points[0].setCoords(pointsPrime[0]);
                points[1].setCoords(pointsPrime[1]);
                points[2].setCoords(pointsPrime[2]);

                //BACKWARD

                pointsPrime[0].setCoords(armOriginPoint);
                pointsPrime[1].setCoords(getNextPrime(pointsPrime[0], points[1], armLengths[0]));
                pointsPrime[2].setCoords(getNextPrime(pointsPrime[1], points[2], armLengths[1]));

                points[0].setCoords(pointsPrime[0]);
                points[1].setCoords(pointsPrime[1]);
                points[2].setCoords(pointsPrime[2]);

                i++;
            }
        }
        else{

            Vector vector = new Vector(goal, armOriginPoint);

            points[0].setCoords(armOriginPoint);
            vector.setLength(armLengths[0]);
            points[1].setCoords(vector.getTerminal());
            vector.setLength(armLengths[1] + vector.getMagnitude());
            points[2].setCoords(vector.getTerminal());


        }

        //Doesn't work and not worth fixing.
        points[1].setCoords(reflectionFix());

    }

    public double getArmMiddleAngle(){
        return armSegToVector(0).getAngleBetweenOther(armSegToVector(1));
    }

    public double getArmBaseAngle(){
        return armSegToVector(0).getAngleBetweenOther(new Vector(new Point(1, 0), new Point()));
    }

    private Vector armSegToVector(int segNumber){
        return new Vector(points[segNumber + 1], points[segNumber]);
    }

    public Point getNextPrime(Point origin, Point cast, double segLength){

        Vector vector = new Vector(cast, origin);
        vector.setLength(segLength);

        return vector.getTerminal();

    }

    //Doesn't work and not worth fixing.
    public Point reflectionFix(){

        if(points[1].getYRelative(armOriginPoint) <= 0 || !goalIsReachable()){
            return points[1];
        }

        double x = points[2].getXRelative(armOriginPoint);
        double y = points[2].getYRelative(armOriginPoint);
        double a = points[1].getXRelative(armOriginPoint);
        double b = points[1].getYRelative(armOriginPoint);
        double h = new Vector(armOriginPoint, points[2]).getMagnitude();

        return new Point(
                (a * (2 * Math.pow( x/h,2) - 1) + 2*b * (x*y) / Math.pow(h,2)) + armOriginPoint.getX(),
                (-b * (2 * Math.pow( x/h,2) - 1) + 2*a * (x*y) / Math.pow(h,2)) + armOriginPoint.getY()
        );
    }

    public Point getPointInArm(int point){
        return points[point];
    }

    public double getMaxReach() {
        return maxReach;
    }
}
