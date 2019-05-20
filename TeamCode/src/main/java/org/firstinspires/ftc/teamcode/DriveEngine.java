package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

abstract class DriveEngine {
    ArrayList<DcMotor> motors = new ArrayList<>();

    Sensors sensors;

    static double effectiveWheelDiameter = 6;

    private double spinAve,rAve,thetaAve;

    /**
     * Forward is compared to spinAngle(), which is the angle the gyroscope has moved from its initial position.
     * So forward is also measured relative to the gyroscope's initial position.
     * Forward is generally the angle the gyroscope tries to match up with.
     * When spinning to a new position, forward retains the last position for reference.
     */
    private double forward;
    /**
     * The default x-axis on the robot points in the direction of the 0 motor.
     * Initial angle rotates the x-axis and sets the new default.
     * It changes where you can think of the front of the robot.
     * Initial angle is relative to the direction of the 0 motor.
     */
    private double initialAngle;
    /**
     * DriveAngle is relative to initialAngle.
     * If the driveEngine is directed to move in the positive x direction, it will move in the direction of driveAngle.
     */
    private double driveAngle;
    /**
     * FieldAngle is the rotation of the trueXY coordinate system relative to initialAngle. I think.
     * If the robot is angled diagonally relative to the field to start, fieldAngle could be set to pi/4.
     */
    private double fieldAngle;

    ElapsedTime timer;
    Telemetry telemetry;
    SmoothingType currentSmoothing = SmoothingType.Linear;

    double[] blackValues = new double[3];

    double trueX;
    double trueY;

    DriveEngine(Telemetry telemetry, Sensors sensors) {
        this.telemetry = telemetry;
        this.sensors = sensors;

        timer = new ElapsedTime();
    }

    //Used in the FakeDriveEngine
    DriveEngine() {
    }

    //Potential drive values
    private ArrayList<DriveValuePacket> potentialDrivePackets = new ArrayList<>();

    //Answers the question: which drive value do we choose?
    private ArrayList<Integer> precedences = new ArrayList<>();


    private ArrayList<Double> smoothRList = new ArrayList<>();
    private ArrayList<Double> smoothThetaList = new ArrayList<>();
    private ArrayList<Double> smoothSpinList = new ArrayList<>();

    enum SmoothingType {
        Exponential,
        Linear
    }


    /**
     *  Drives with zero power and high precedence
     */
    void stop(){
        drive(2,0,false,0);
    }

    /**
     * Drives with one double input.
     * @param spin: The rotation speed of the robot.
     */
    void rotate(double spin){
        drive(spin);
    }


    /**
     * When only numbers are provided, we assume op is false.
     * @param args: x, y, and spin
     */
    void drive(double... args) {
        drive(false,args);
    }

    /**
     * By default, the precedence is 0, and there is no smoothing.
     * @param op means overpowered: When op, a motor is maxed out if the overall power > .9
     * @param args: x, y, and spin
     */
    void drive(boolean op, double ... args){drive(0, 0, op, args);}

    /**
     * By default, only r is smoothed; smoothTheta and smoothSpin are false.
     * @param precedence: Higher precedence drive values are taken over low ones.
     * @param rSeconds: The number of seconds to reach the true ga
     * @param op means overpowered: When op, a motor is maxed out if the overall power > .9
     * @param args: x, y, and spin
     */
    void drive(int precedence, double rSeconds, boolean op,  double ... args)
    {
        drive(precedence, rSeconds, op, false, false,  args);
    }

    /**
     * currentSmoothing is the default smoothing.
     * @param precedence: Higher precedence drive values are taken over low ones.
     * @param rSeconds: The number of seconds to reach the input value
     * @param op means overpowered: When op, a motor is maxed out if the overall power > .9
     * @param smoothSpin: Whether spinning should be smoothed or taken as is
     * @param smoothTheta: Whether the direction of motion should be smoothed or taken as is.
     * @param args: x, y, and spin
     */
    void drive(int precedence, double rSeconds, boolean op, boolean smoothTheta, boolean smoothSpin, double... args)
    {
        drive(precedence, rSeconds, op, currentSmoothing, smoothTheta, smoothSpin, args);
    }

    /**
     * currentSmoothing is the default smoothing.
     * @param precedence: Higher precedence drive values are taken over low ones.
     * @param rSeconds: The number of seconds to reach the input value
     * @param op means overpowered: When op, a motor is maxed out if the overall power > .9
     * @param type: Exponential or Linear smoothing
     * @param smoothSpin: Whether spinning should be smoothed or taken as is
     * @param smoothTheta: Whether the direction of motion should be smoothed or taken as is.
     * @param args: x, y, and spin
     */
    void drive(int precedence, double rSeconds, boolean op, SmoothingType type, boolean smoothTheta, boolean smoothSpin, double... args)
    {
        drive(precedence, new DriveValuePacket(rSeconds, op, type, smoothTheta, smoothSpin, args));
    }

    /**
     * This method adds drive values to the list of potential drive values.
     * It doesn't actually send powers to the motors.
     * Explicit precedence is followed.
     * Ties are broken like such: non-zero values are chosen over stopping.
     * More recent values are chosen over old ones.
     *
     * @param precedence The precedence of the drive packet
     * @param potentialDrivePacket: A packet of drive values
     */
    private void drive(int precedence, DriveValuePacket potentialDrivePacket) {
        //If we've already logged power values this loop
        if(precedences.size() != 0){
            //If our precedence is too low, we break out, no more math needed.
            if (precedence < MyMath.max(precedences))
                return;
            //Driving with non-zero values takes precedence over stopping,
            //Unless stopping has explicit precedence.
            //If the values are zero, we break out.
            if(precedence == MyMath.max(precedences))
                if(MyMath.absoluteMax(potentialDrivePacket.xys) == 0)
                    return;
        }
        //If we've made it to this point, we want to keep our drive values.
        //We save the precedence
        precedences.add(precedence);
        //We save op and xys into an array.

        //We put our drive values in the first spot in the potential ArrayList.
        potentialDrivePackets.add(0, potentialDrivePacket);
    }

    /**
     * This method should be called once per loop.
     * It calls drive(), which updates the motor powers.
     * It clears some objects for use in the next loop.
     */
    void update(){
        //If all is working, this line should appear.
        //It might not if the update method is left out of an OpMode.
        telemetry.addLine("updating");

        //Once per loop, we update the motor powers.
        double[] motorPowers = processMotorPowersFromDriveValues(processPotentials());

        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(motorPowers[i]);
            telemetry.addData("motor" + i + " power", motorPowers[i]);
        }


        //We prepare for the next loop by clearing one-loop lists and counters.
        precedences.clear();
        potentialDrivePackets.clear();
        updateTrueDistances();
        moveRobotOnScreen();
    }

    /**
     * This method actually sends powers to the motors.
     * This method should call processPotentials() to retrieve x, y, and spin.
     */
    abstract double[] processMotorPowersFromDriveValues(double[] driveValues);

    /**
     * This method selects the proper drive values from the potentials.
     * It processes x, y and spin for use in drive().
     */
    private double[] processPotentials()
    {
        //If we haven't been given any drive values, we stop.
        if(potentialDrivePackets.size() == 0){
            stop();
            return processPotentials();
        }

        DriveValuePacket dvp = potentialDrivePackets.get(0);
        double x = dvp.x;
        double y = dvp.y;
        double spin = dvp.spin;


        if(MyMath.absoluteMax(x, y) == 0) {  //If we are to stop,
            smoothThetaList.clear();         //Reset our direction: no delay
            MyMath.fill(smoothRList, 0);  //Stop immediately
        }
        if(spin == 0)
            MyMath.fill(smoothSpinList, 0);

        double xPrime = x * Math.cos(driveAngle) - y * Math.sin(driveAngle); //adjust for angle
        double yPrime = x * Math.sin(driveAngle) + y * Math.cos(driveAngle);

        x = xPrime;
        y = yPrime;

        double r = Math.hypot(x,y);
        r = Math.min(r, 1);

        double theta = Math.atan2(y, x);

        double spinSeconds = 4;
        double thetaSeconds = 2/3.;

        switch (dvp.type) {
            case Exponential:
                double rAlpha = Bogg.getAlpha(dvp.rSeconds);
                double spinAlpha = Bogg.getAlpha(spinSeconds);
                double thetaAlpha = Bogg.getAlpha(thetaSeconds);

                rAve = r == 0 ? 0 : rAve + rAlpha * (r - rAve);
                spinAve = spin == 0 ? 0 : spinAve + spinAlpha * (spin - spinAve);
                thetaAve += thetaAlpha * MyMath.loopAngle(theta, thetaAve);
                break;

            case Linear:

                MyMath.trimFromFront(smoothRList, (int) Math.round(dvp.rSeconds / Bogg.averageClockTime) -1);
                MyMath.trimFromFront(smoothSpinList, (int) Math.round(4 / Bogg.averageClockTime) -1);
                MyMath.trimFromFront(smoothThetaList, (int) Math.round(.66 / Bogg.averageClockTime) -1);

                smoothRList.add(r);
                smoothSpinList.add(spin);
                smoothThetaList.add(theta);

                rAve = (r == 0) ? 0 : MyMath.ave(smoothRList);
                spinAve = (spin == 0 || !dvp.smoothSpin) ? spin : MyMath.ave(smoothSpinList);
                thetaAve = dvp.smoothTheta ? MyMath.loopAve(smoothThetaList) : theta;
                break;
        }


        telemetry.addData("driveE x", x);
        telemetry.addData("driveE y", y);
        telemetry.addData("driveE rotate", spin);

        blackValues = new double[]{x,y,spin};

        return new double[]{dvp.op? 1:0,
                Math.cos(thetaAve) * rAve,
                Math.sin(thetaAve) * rAve,
                spinAve};
    }

    void setInitialPosition(double x, double y)
    {
        trueX = x;
        trueY = y;
        lastCheckpoint = new double[]{trueX, trueY, forward};
    }
    void setInitialAngle(double angle)
    {
        driveAngle = initialAngle = fieldAngle = angle;
    }
    void setFieldAngle(double angle){fieldAngle = angle;}

    /**
     * Also known as pressing x
     */
    void resetFieldHeadingToRobotHeading()
    {
        fieldAngle = initialAngle + spinAngle();
    }

    /**
     * Should be called once per loop in fixed-forward driving.
     */
    void orientRobotDirectionToField()
    {
        driveAngle = MyMath.loopAngle(fieldAngle, spinAngle());
    }


    /**
     * Sets all motors to float instead of brake.
     * This is useful when testing positions for the encoders.
     */
    void floatMotors()
    {
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        updateTrueDistances();
    }

    private int checkpointsLeft = 0;
    private ArrayList<String> keyList = new ArrayList<>();

    boolean moveOnPath(double[] ... args){
        return moveOnPath(Positioning.Relative, args);
    }
    boolean moveOnPath(String key, double[] ... args){
        return moveOnPath(key, Positioning.Relative, args);
    }

    boolean moveOnPath(String key, Positioning positioning, double[] ... args){
        if(keyList.contains(key))
            return true;
        if(moveOnPath(positioning, args)){
            keyList.add(key);
            return true;
        }
        return false;
    }

    enum Positioning
    {
        Absolute,
        Relative
    }


    void updateCheckpoint()
    {
        lastCheckpoint = new double[]{trueX, trueY};
    }
    private double[] lastCheckpoint = new double[]{trueX, trueY};

    /**
     *@param positioning is the type of positioning system.
     *      Absolute is relative the origin point.
     *      Relative is relative to robot's last position.
     *      Note: when using relative coordinates, a key should probably be used.
     * @param args is a list of a set of points. Each set looks like this: {3,4}
     *             and is encapsulated in a double[].
     *             The double[]s are added with commas to separate them.
     * @return if all checkpoints have been completed.
     */
    boolean moveOnPath(Positioning positioning, double[] ... args)
    {
        if(checkpointsLeft == 0) {
            checkpointsLeft = args.length;
        }

        int c = args.length - checkpointsLeft;
        telemetry.addData("checkpoints count", c);

        double targetAngle = forward;
        double currentAngle = spinAngle();
        switch (args[c].length)
        {
            case 1:
                targetAngle = forward + args[c][0];

                if(Math.abs(MyMath.loopAngle(targetAngle, currentAngle)) < MyMath.radians(2)) {
                    forward = targetAngle;
                    sumSpinError = 0;
                    checkpointsLeft--;
                    break;
                }
                rotate(angularVelocityNeededToFace(targetAngle));
                break;
            case 3:
                targetAngle = forward + args[c][2];

            case 2:
                double[] point = args[c];

                double deltaX=0, deltaY=0, trueDeltaX, trueDeltaY;

                double fieldToRobotRotation = fieldAngle - currentAngle - driveAngle;

                switch (positioning)
                {
                    case Absolute:
                        //We know the difference between the target position and the robot's position in absolute coordinates
                        trueDeltaX = point[0] - trueX;
                        trueDeltaY = point[1] - trueY;
                        //But we need to find out how the robot sees it; if it needs to move forward or backward.
                        deltaX =  trueDeltaX * Math.cos(fieldToRobotRotation) - trueDeltaY * Math.sin(fieldToRobotRotation);
                        deltaY =  trueDeltaX * Math.sin(fieldToRobotRotation) + trueDeltaY * Math.cos(fieldToRobotRotation);
                        break;
                    case Relative:
                        //This part transfers the relative vector to where it should be: relative to robot forward.
                        //We are finding trueXY (absolute) coordinates for the next checkpoint.
                        double previousAngle = forward + driveAngle - fieldAngle; //angle between robot forward and field forward
                        double truePointX = lastCheckpoint[0] + point[0] * Math.cos(previousAngle) - point[1] * Math.sin(previousAngle);
                        double truePointY = lastCheckpoint[1] + point[1] * Math.sin(previousAngle) + point[0] * Math.cos(previousAngle);

                        //Then we find deltaX and deltaY relative to the robot's current position.
                        //Note that the last part was concerned with the robot's previous position.
                        trueDeltaX = truePointX - trueX;
                        trueDeltaY = truePointY - trueY;
                        deltaX =  trueDeltaX * Math.cos(fieldToRobotRotation) - trueDeltaY * Math.sin(fieldToRobotRotation);
                        deltaY =  trueDeltaX * Math.sin(fieldToRobotRotation) + trueDeltaY * Math.cos(fieldToRobotRotation);
                }

                if(motors.size() == 2)
                {
                    //Remember that 0 is oriented with motor0, so you can only move on the x axis.
                    //We want deltaY to be zero. When it is, the arctangent term is also zero.
                    targetAngle = Math.atan2(deltaY, deltaX);
                }

                telemetry.addData("deltaX", deltaX);
                telemetry.addData("deltaY", deltaY);

                double r = Math.hypot(deltaX, deltaY);

                double[] drive = move(deltaX, deltaY);
                double spin = angularVelocityNeededToFace(targetAngle);

                if(r <= .75){
                    if(args[c].length == 3) {
                        sumSpinError = 0;
                    }
                    forward = targetAngle;
                    checkpointsLeft--;
                    break;
                }
                else {
                    //smooth the driving when revving to a high speed, then reset the average to 0.
                    if(Math.hypot(drive[0],drive[1]) > .3)
                        drive(0,1,false, false, false,
                                drive[0], drive[1], spin);
                    else
                        drive(drive[0], drive[1], spin);
                }
                break;
        }

        return checkpointsLeft == 0;
    }



    double angularVelocityNeededToFaceForward(){
        return angularVelocityNeededToFace(forward);
    }

    private double sumSpinError = 0;
    private double lastSpinError = 0;
    private double lastSpinTime = 0;

    double sP = .16; // .16 per radian
    double sI = 8; //Time to correct past error
    double sD = .7; //fully account for this much time in the future at current error decreasing rate

    /**
     * This method uses PID control to rotate the robot to a certain angle,
     *  or keep the robot facing forward.
     * @param angle the robot should face.
     * @return the rotating power needed to reach that angle
     */
    private double angularVelocityNeededToFace(double angle)
    {
        //    u(t) = MV(t) = P *( e(t) + 1/I* integral(0,t) (e(tau) *dtau) + 1/D *de(t)/dt )
        //    where
        //    P is the proportional gain, a tuning parameter,
        //    I is the time to correct past error, a tuning parameter,
        //    D is the time the equation predicts to correct for future error, a tuning parameter,
        //    e(t) = set point - current point
        //    t is the time or instantaneous time
        //    tau is the variable of integration (takes on values from time zero to the present t).
        //power per degree
        double i = sI;

        double e = MyMath.loopAngle(angle, spinAngle());
        double de = e - lastSpinError; //change in angle
        double t = timer.seconds();
        double dt = t - lastSpinTime;  //change in t
        telemetry.addData("de/dt", de/dt);
        sumSpinError += e * dt;        //cumulative error

        if(Math.abs(e) > MyMath.radians(5)) {
            sumSpinError = 0;
            i = 10000;
        }
        telemetry.addData("sum Spin Error", sumSpinError);

        double power = sP * (e  +  1/ i * sumSpinError +  sD * de/dt);
        telemetry.addData("correction Power", power);
        //e is current error, sumSpinError is the integral, de/dt is the derivative

        lastSpinError = e;
        lastSpinTime = t;
        return power;
    }

    void resetCorrectionForwardToRobotForward()
    {
        forward = spinAngle();
        sumSpinError = 0;
    }


    private double lastR = 0;
    private double lastT = 0;
    double mP = .02; //power per inch
    double mD = 0.5;  //fully account for this much time in the future at current error decreasing rate

    private double[] move(double deltaX, double deltaY)
    {
        //    u(t) = MV(t) = P *( r(t) + 1/D *dr(t)/dt )
        //    where
        //    P is the proportional gain, a tuning parameter,
        //    D is the time the equation predicts to correct for future error, a tuning parameter,
        //    r(t) = set point - current point
        //    t is the time or instantaneous time

        double r = Math.hypot(deltaX, deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        telemetry.addData("radius", r);
        double dr = r - lastR;
        double t = timer.seconds();
        double dt = t - lastT;
        double drdt = dr / dt;


        telemetry.addData("theta", theta);

        double power = mP * (r +  mD * drdt);
        telemetry.addData("moveD term", mD * drdt);
        telemetry.addData("power", power);
        telemetry.addData("moveD / power",mD * drdt / power);

        if(power > .5) power = .5;
        //Don't worry; it's smoothed

        lastR = r;
        lastT = t;

        return new double[]{Math.cos(theta) * power,
                            Math.sin(theta) * power};
    }



    //What this should do is change the robot's center of rotation.
    abstract void orbit(double radius, double angle, double speed);


    /**
     * A method for finding distance travelled in a direction.
     * @param angle, the angle in radians.
     * @return the total distance travelled in the direction of angle.
     */
    double getDistance(double angle)
    {
        return trueX * Math.cos(angle) + trueY * Math.sin(angle);
    }

    /**
     * A method for finding distance travelled.
     * This method should assume the robot does not rotate.
     * X is in the direction of motor0, or right.
     *
     * @return inches travelled in the x direction
     */
    abstract double xDist();

    /**
     * A method for finding distance travelled.
     * This method should assume the robot does not rotate.
     * Y is in the direction of pi/2, or forward.
     *
     * @return inches travelled in the y direction
     */
    abstract double yDist();

    /**
     * This method uses the IMU's gyroscope to find the heading.
     * If we are not using the IMU, this method assumes an OmniWheelDriveEngine.
     * This method may need to be overridden if this is false.
     * @return The angle the robot has spun.
     */
    double spinAngle()
    {
        if(sensors.usingImu)
            return sensors.getImuHeading();

        else
        {
            double sum = 0;
            for (int i = 0; i < motors.size(); i++)
                sum += motors.get(i).getCurrentPosition();

            return sum / motors.size();
        }
    }

    double lastX = 0;
    double lastY = 0;

    private boolean trackDistances = true;
    void disableDistanceTracking(){trackDistances = false;}
    void enableDistanceTracking(){trackDistances = true;}

    /**
     * This method updates trueX and trueY.
     * It also reports the motor positions to the screen.
     * This method is essential for absolute positioning.
     */
    void updateTrueDistances()
    {
        for (int i = 0; i < motors.size(); i++) {
            telemetry.addData("motor " + i + " position", motors.get(i).getCurrentPosition());
        }

        double x = xDist();
        double y = yDist();
        double spin = spinAngle();
        double dX = x - lastX;
        double dY = y - lastY;

        double xPrime = dX * Math.cos(spin) - dY * Math.sin(spin);
        double yPrime = dX * Math.sin(spin) + dY * Math.cos(spin);

        if(trackDistances) {
            trueX += xPrime;
            trueY += yPrime;
        }

        lastX = x;
        lastY = y;
    }

    /**
     * This method moves the onscreen robot.
     * You can change the x and y multipliers to align the robot coordinate system
     * to the phone coordinate system.
     */
    void moveRobotOnScreen()
    {
        Screen.moveRobotToPosition(trueX, trueY, spinAngle());
    }
}
