package Filters;

public class LowPassF {
    private final double alpha = 0.7;
    private double x;

    public LowPassF(int u){
        x = u;
    }

    public void execute(double u){
        x = x*alpha + (1-alpha)*u;
        //return (int) Math.round(x);
    }
    public int getEstimate(){
        return (int) Math.round(x);
    }

}
