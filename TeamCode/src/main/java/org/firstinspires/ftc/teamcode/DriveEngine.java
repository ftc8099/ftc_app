//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import java.util.ArrayList;
//
//class DriveEngine {
//    ArrayList<DcMotor> motors = new ArrayList<>();
//    private double spinAve,rAve,thetaAve;
//
//    Telemetry telemetry;
//
//    //Potential drive values
//    private ArrayList<DriveValuePacket> potentialDrivePackets = new ArrayList<>();
//    private ArrayList<Double> smoothRList = new ArrayList<>();
//    private ArrayList<Double> smoothThetaList = new ArrayList<>();
//    private ArrayList<Double> smoothSpinList = new ArrayList<>();
//    double driveAngle;
//
//    void update(){
//        double[] motorPowers = processPotentials();
//        for (int i = 0; i < motors.size(); i++) {
//            motors.get(i).setPower(motorPowers[i]);
//            telemetry.addData("motor" + i + " power", motorPowers[i]);
//        }
//    }
//
//    private double[] processPotentials()
//    {
//        //If we haven't been given any drive values, we stop.
//        if(potentialDrivePackets.size() == 0){
////            stop();
//            return processPotentials();
//        }
//
//        DriveValuePacket dvp = potentialDrivePackets.get(0);
//        double x = dvp.x;
//        double y = dvp.y;
//        double spin = dvp.spin;
//
//
//        if(MyMath.absoluteMax(x, y) == 0) {  //If we are to stop,
//            smoothThetaList.clear();         //Reset our direction: no delay
//            MyMath.fill(smoothRList, 0);  //Stop immediately
//        }
//        if(spin == 0)
//            MyMath.fill(smoothSpinList, 0);
//
//        double xPrime = x * Math.cos(driveAngle) - y * Math.sin(driveAngle); //adjust for angle
//        double yPrime = x * Math.sin(driveAngle) + y * Math.cos(driveAngle);
//
//        x = xPrime;
//        y = yPrime;
//
//        double r = Math.hypot(x,y);
//        r = Math.min(r, 1);
//
//        double theta = Math.atan2(y, x);
//
//        double spinSeconds = 4;
//        double thetaSeconds = 2/3.;
//
////        double rAlpha = Bogg.getAlpha(dvp.rSeconds);
////        double spinAlpha = Bogg.getAlpha(spinSeconds);
////        double thetaAlpha = Bogg.getAlpha(thetaSeconds);
////
////        rAve = r == 0 ? 0 : rAve + rAlpha * (r - rAve);
////        spinAve = spin == 0 ? 0 : spinAve + spinAlpha * (spin - spinAve);
////        thetaAve += thetaAlpha * MyMath.loopAngle(theta, thetaAve);
//
//        telemetry.addData("driveE x", x);
//        telemetry.addData("driveE y", y);
//        telemetry.addData("driveE rotate", spin);
//
//        return new double[]{dvp.op? 1:0,
//                Math.cos(thetaAve) * rAve,
//                Math.sin(thetaAve) * rAve,
//                spinAve};
//    }
//}
