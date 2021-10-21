package frc.robot.subsystems;

public class StateClass 
{
    public String action;
    public float p1;
    public float p2;
    public float p3;
    public float p4;
    public boolean p5;

    public StateClass(String actionIn, float p1In, float p2In, float p3In, float p4In, boolean p5In)
    {
        this.action = actionIn;
        this.p1 = p1In;
        this.p2 = p2In;
        this.p3 = p3In;
        this.p4 = p4In;
        this.p5 = p5In;
    }
}
