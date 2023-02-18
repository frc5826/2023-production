package frc.robot.fabrik;

public class Vector {

    private Point terminal, initial;

    public Vector(Point terminal, Point initial){
        this.terminal = terminal;
        this.initial = initial;
    }

    public Vector(){
        this.terminal = new Point(0,0);
        this.initial = new Point(0,0);
    }

    public void normalize(){
        double length = Math.sqrt( Math.pow(terminal.getX() - initial.getX(), 2) + Math.pow(terminal.getY() - initial.getY(), 2) );
        double difX = terminal.getX() - initial.getX();
        double difY = terminal.getY() - initial.getY();

        terminal.setCoordsRelative(initial, difX / length, difY / length);
    }

    public void setLength(double length){
        normalize();
        terminal.setCoords(terminal.getXRelative(initial) * length + initial.getX(), terminal.getYRelative(initial) * length + initial.getY());
    }

    public Vector getInverted(){
        Vector v = new Vector(terminal, initial);
        v.setLength(-v.getMagnitude());
        return v;
    }

    public double getMagnitude(){
        return Math.sqrt(Math.pow(terminal.getXRelative(initial), 2) + Math.pow(terminal.getYRelative(initial), 2));
    }

    public double getAngleBetweenOther(Vector other){
        return Math.acos((this.getRelative().getX() * other.getRelative().getX() + this.getRelative().getY() * other.getRelative().getY()) / (this.getMagnitude() * other.getMagnitude()));
    }

    public Point getTerminal(){
        return terminal;
    }

    public Point getInitial(){
        return initial;
    }

    public Point getRelative(){
        return new Point(terminal.getXRelative(initial), terminal.getYRelative(initial));
    }

    public Vector moveToOrigin(){
        return new Vector(this.getRelative(), new Point());
    }

}
