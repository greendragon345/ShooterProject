package frc.robot.subsystems.Shood;

import java.util.ArrayList;

import frc.robot.Constants;

public class Utils {

    public int searchInTable(double x) {
        for (int i = 0; i < Constants.LOOK_UP_TABLE.length; i++) {
            if (Constants.LOOK_UP_TABLE[0][i] >= x) {
                return i;
            }
        }
        return Constants.LOOK_UP_TABLE.length;
    }

    public double interp(double x) {
        int I = searchInTable(x);
        if (I == 0) {
            return 0;
        }
        if (I == Constants.LOOK_UP_TABLE.length) {
            return 0;
        }
        double tot = Constants.LOOK_UP_TABLE[0][I] - Constants.LOOK_UP_TABLE[0][I - 1];
        double part = Constants.LOOK_UP_TABLE[0][I] - x;
        return part / tot;

    }

    public double calcVelocity(double x) {
        if (searchInTable(x) == 0) {
            return Constants.LOOK_UP_TABLE[1][0];
        }
        if (searchInTable(x) == Constants.LOOK_UP_TABLE.length) {
            return Constants.LOOK_UP_TABLE[1][Constants.LOOK_UP_TABLE.length - 1];
        }
        double vel = Constants.LOOK_UP_TABLE[1][searchInTable(x)];
        double downvel = Constants.LOOK_UP_TABLE[1][searchInTable(x) - 1];
        double tot = vel - downvel;
        return vel - interp(x) * tot;

    }

    public double calcAngle(double x) {
        if (searchInTable(x) == 0) {
            return Constants.LOOK_UP_TABLE[2][0];
        }
        if (searchInTable(x) == Constants.LOOK_UP_TABLE.length) {
            return Constants.LOOK_UP_TABLE[2][Constants.LOOK_UP_TABLE.length - 1];
        }

        double angle = Constants.LOOK_UP_TABLE[2][searchInTable(x)];
        double downangle = Constants.LOOK_UP_TABLE[2][searchInTable(x) - 1];
        double tot = angle - downangle;
        return angle - interp(x) * tot;

    }

    public double[] calculate(double dis) {
        double[] db = new double[2];
        db[0] = calcVelocity(dis);
        db[1] = calcAngle(dis);
        return db;
    }

    public static double todegsin(double degr) {
        return Math.toDegrees(Math.sin(Math.toRadians(degr)));
    }

    public static double toDegCos(double degr) {
        return Math.toDegrees(Math.cos(Math.toRadians(degr)));

    }

    public double[] decider(double disitan) {
        double maxspeed = 70;
        double speedid =0;
        ArrayList<Double> posblsoultion = new ArrayList<Double>(calcanglNLT(disitan));//made the arraylist a singel dimesion one

        int i = 0;
        int maxi = 0;
        // double sum = posblsoultion.get(0)* Constants.shootratio
        //         + posblsoultion.get(1) * Constants.hoodratio;
        for (int j = 0; j < posblsoultion.size(); j+=2) {
            double speedatpoint = get_velocity_and_angle(posblsoultion.get(j),Math.toRadians(posblsoultion.get(j+1)), disitan)[0];
            double degreeatpoint = get_velocity_and_angle(posblsoultion.get(j), Math.toRadians(posblsoultion.get(j+1)), disitan)[1];
            double degrindeg = Math.toDegrees(degreeatpoint);
            // System.out.println("shooter speed: "+posblsoultion.get(j));
            // System.out.println("shooter angle: "+posblsoultion.get(j+1));
            if(Math.abs(speedatpoint*Math.sin(degreeatpoint))<Constants.minspeed && Math.abs(speedatpoint* Math.cos(degreeatpoint))<Math.abs(maxspeed)){
                maxspeed = speedatpoint * Math.cos(degreeatpoint);
                i = j;
            }
        // double sum2 =
        // +posblsoultion.get(j)*Constants.shootratio+posblsoultion.get(j+1)*Constants.hoodratio;
        // if(sum>sum2){
        // maxi =i;
        // }
        }
        double[] velang = { posblsoultion.get(i), posblsoultion.get(i+1)};
        return velang;

    }
    public double[] get_velocity_and_angle(double speed,double angle,double distance){
        double H = calcH(speed, angle);
        double T = 2 * Math.sqrt(2 * H / Constants.g);
        double angl1 = angle / 2 + Math.PI / 4;
        double Va = speed * Math.cos(angle) / Math.sqrt(1 + Constants.K * Math.pow(speed, 2) *
                (Math.sin(angle) + Math.pow(Math.cos(angle), 2) * Math.log(Math.sin(angl1) / Math.cos(angl1))));
        double L = Va * T;
        double Xa = Math.sqrt(L * H * (Math.cos(angle) / Math.sin(angle)));
        double a1 = L/Xa;
        double a2 = (L-2*Xa)/H;
        double a3 = Xa*Math.pow((2-a1),-1);
        double eqp1 = 1-2*a1+Math.pow(a1,2);
        double eqp2 = Math.pow((distance/a3),2)-2*(distance/a3)+1;
        double angletangent = ((eqp1/eqp2)-1)/a2;
        double angleinrad = Math.atan(angletangent);
        double speedatpoint = (speed* Math.cos(angle))/(Math.cos(angleinrad)*Math.sqrt(1+(Constants.K*Math.pow(speed,2)*Math.pow(Math.cos(angle),2)*(anglefunction(angle)-anglefunction(angleinrad)))));
        double[] speedandangle =  {speedatpoint,angleinrad};
        return speedandangle;




    }
    public double anglefunction(double angle){
        double logangle = angle / 2 + Math.PI / 4;
        double function = (Math.sin(angle)/Math.pow(Math.cos(angle),2))+Math.log(Math.tan(logangle));
        return function;
        
    }

    public ArrayList<Double> calcanglNLT(double dist) {
        ArrayList<Double> d1 = new ArrayList<Double>();
        for (double i = 0; i <89; i += 0.1) {
            double minimumspeed = 2.0;

            double speed = calcvelNLT(Math.sqrt(Constants.maxH * Constants.g * 2 /(
            Math.pow(Math.sin(Math.toRadians(89.9)),2) - Constants.maxH*Constants.g * Constants.K*Math.sin(Math.toRadians(89.99)))), Math.toRadians(i), dist,minimumspeed);
            minimumspeed = speed;
            while(minimumspeed>0){
                    d1.add(speed);
                    d1.add(i);
                    speed = calcvelNLT(Math.sqrt(Constants.maxH * Constants.g * 2 /(
                    Math.pow(Math.sin(Math.toRadians(89.9)),2) - Constants.maxH*Constants.g * Constants.K*Math.sin(Math.toRadians(89.99)))), Math.toRadians(i), dist,minimumspeed+0.1);
                    minimumspeed = speed;
                

                
            }
        }
        return d1;
    }

    public double calcvelNLT(double maxspeed, double deg, double dist,double minimumspeed) {

        for (double i = minimumspeed; i < 52; i += 0.1) { // kinda think we should change the max min speed
                                                                       // to just speeds we know the robot can reach.
                                                                       //jhony:the maximmup speed of the falcon is very high 
                                                                       //BTW the fulcon is accelratin isnt it , does he have a maximum speed?
            double distV = calcdist(deg, i);
            if (Math.abs(distV - dist) <= 0.01 && calcH(i, deg) < Constants.maxH) { // added the H thing to the if
                return i;
            }

        }
        return 0;

    }

    public double calcH(double speed, double angle) { // wrote H as a function
        double H = 
        (Math.pow(speed, 2) * Math.pow(Math.sin(angle), 2)) / ((Constants.g
                * (2 + ((Constants.K * Math.pow(speed, 2)) * (Math.sin(angle))))));
        return H;
    }

    public double calcdist(double angle, double speed) {
        double H = calcH(speed, angle);
        double T = 2 * Math.sqrt(2 * H / Constants.g);
        double angl1 = angle / 2 + Math.PI / 4;
        double Va = speed * Math.cos(angle) / Math.sqrt(1 + Constants.K * Math.pow(speed, 2) *
                (Math.sin(angle) + Math.pow(Math.cos(angle), 2) * Math.log(Math.sin(angl1) / Math.cos(angl1))));
        double L = Va * T;
        double Xa = Math.sqrt(L * H * (Math.cos(angle) / Math.sin(angle)));
        return quadeq(-H, H * L - ((L - 2 * Xa) * Constants.hieght), -1*Constants.hieght * (Math.pow(Xa, 2)));

    }

    public double quadeq(double a, double b, double c) {
        double firstroot, secondroot;

        double det = b * b - 4 * a * c;

        if (det > 0) {

            // two real and distinct roots
            firstroot = (-b + Math.sqrt(det)) / (2 * a);
            secondroot = (-b - Math.sqrt(det)) / (2 * a);
           
            return Math.max(firstroot, secondroot);
        } else {
            return 0;
        }

    }
    public static double convertshooterspeedtoballspeed(double shooterspeed){
        return shooterspeed*Constants.shooterexitspeedgradient+Constants.shooterexitspeedintercept;
    }

}
