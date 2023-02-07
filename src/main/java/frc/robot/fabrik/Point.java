package frc.robot.fabrik;

public class Point {

    private double x, y;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(){
        this.x = 0;
        this.y = 0;
    }

    public void setCoords(Point point){
        this.x = point.getX();
        this.y = point.getY();
    }

    public void setCoords(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setCoordsRelative(Point origin, double x, double y){
        this.x = x + origin.getX();
        this.y = y + origin.getY();
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getXRelative(Point origin){
        return x - origin.getX();
    }

    public double getYRelative(Point origin){
        return y - origin.getY();
    }

    public double getDistanceTo(Point other){
        return Math.sqrt(Math.pow(this.getXRelative(other), 2) + Math.pow(this.getYRelative(other), 2));
    }

}
