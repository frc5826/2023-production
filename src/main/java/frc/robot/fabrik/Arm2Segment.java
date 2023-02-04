package frc.robot.fabrik;

import java.util.Arrays;

enum MovDir{
    UP,
    DOWN,
    FORWARD,
    BACKWARD
}

public class Arm2Segment {

    private double[] armLengths;
    private Point[] points = new Point[3];
    private Point goal;

    private double maxReach;
    private double minReach;

    private final Point armOriginPoint;

    private double errorMargin;

    public Arm2Segment(double[] armLengths, double errorMargin){
        this.armLengths = armLengths;
        for(int x = 0; x < points.length; x++){
            points[x] = new Point();
        }
        findPointsFromEncoders();

        goal = new Point();
        armOriginPoint = new Point();

        this.errorMargin = errorMargin;

        maxReach = Arrays.stream(armLengths).sum();
        minReach = Math.abs(armLengths[2] - armLengths[1]);
    }

    public void findPointsFromEncoders(){
        //TODO use encoders to find our starting points
        //currently assumes our arm is perfectly vertical, fine for current equations
        points[1].setCoords(0, armLengths[0]);
        points[2].setCoords(0, armLengths[1] + points[1].getY());
    }

    public void setGoal(Point goal){
        this.goal.setCoords(goal);
    }

    public void moveGoal(double x, double y){
        this.setGoal(new Point(
                Math.max( Math.min(x + goal.getX(), maxReach), 0),
                Math.max( Math.min(y + goal.getY(), maxReach), 0)
        ));
    }

    public boolean outOfMargin(){
        return Math.sqrt(Math.pow((goal.getX()-points[2].getX()),2) + Math.pow((goal.getY()-points[2].getY()),2)) > errorMargin;
    }

    public boolean goalIsReachable(){
        return minReach <= new Vector(goal, new Point()).getMagnitude() && new Vector(goal, new Point()).getMagnitude() <= maxReach;
    }

    public void fabrik(){

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

    public Point getPointInArm(int point){
        return points[point];
    }

    public double getMaxReach() {
        return maxReach;
    }
}
