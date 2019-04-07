package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

abstract class DriveEngine {
    ArrayList<DcMotor> motors = new ArrayList<>();

    Sensors sensors;

    static double effectiveWheelDiameter = 6;

    private double spinAve,rAve,thetaAve,forward;
    private double driveAngle;
    private double initialAngle;
    ElapsedTime timer;
    Telemetry telemetry;
    SmoothingType currentSmoothing = SmoothingType.None;

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
        Linear,
        None
    }

    enum SpinSmoothing{
        Off(0),
        Normal(4);

        double seconds;
        SpinSmoothing(double seconds)
        {
            this.seconds = seconds;
        }
    }


    /**
     *  Drives with zero power and high precedence
     */
    void stop(){
        drive(2,false, 0);
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
     * The default precedence is 0.
     * @param op means overpowered: When op, a motor is maxed out if the overall power > .9
     * @param args: x, y, and spin
     */
    void drive(boolean op, double ... args){drive(0, op, args);}

    void drive(int precedence, boolean op, double ... args)
    {
        drive(precedence, op, SmoothingType.None, 0, args);
    }

    void drive(int precedence, boolean op, SmoothingType type, double rSeconds, double ... args)
    {
        drive(precedence, op, type, rSeconds,false, false,  args);
    }

    void drive(int precedence, boolean op, SmoothingType type, double rSeconds, boolean smoothTheta, boolean smoothSpin, double... args)
    {
        drive(precedence, new DriveValuePacket(op, type, rSeconds, smoothTheta, smoothSpin, args));
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
    void drive(int precedence, DriveValuePacket potentialDrivePacket) {
        //If we've already logged power values this loop
        if(precedences.size() != 0){
            //If our precedence is too low, we break out, no more math needed.
            if (precedence < MyMath.max(precedences))
                return;
            //Driving with non-zero values takes precedence over stopping,
            //Unless stopping has explicit precedence.
            //If the values are zero, we break out.
            if(precedence == MyMath.max(precedences))
                if(MyMath.absoluteMax(potentialDrivePacket.args) == 0)
                    return;
        }
        //If we've made it to this point, we want to keep our drive values.
        //We save the precedence
        precedences.add(precedence);
        //We save op and args into an array.

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
                smoothRList.add(r);
                smoothSpinList.add(spin);
                smoothThetaList.add(theta);

                MyMath.trimFromFront(smoothRList, (int) Math.round(dvp.rSeconds / Bogg.averageClockTime));
                MyMath.trimFromFront(smoothSpinList, (int) Math.round(4 / Bogg.averageClockTime));
                MyMath.trimFromFront(smoothThetaList, (int) Math.round(.66 / Bogg.averageClockTime));

                rAve = (r == 0) ? 0 : MyMath.ave(smoothRList);
                spinAve = (spin == 0 || !dvp.smoothSpin) ? spin : MyMath.ave(smoothSpinList);
                thetaAve = dvp.smoothTheta ? MyMath.loopAve(smoothThetaList) : theta;
                break;

            case None:
                rAve = r;
                spinAve = spin;
                thetaAve = theta;
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


    void driveAtAngle(double angle)
    {
        driveAngle = angle;
    }

    void resetFieldHeadingToRobotHeading()
    {
        driveAngle = initialAngle + spinAngle();
    }

    void orientRobotDirectionToField()
    {
        driveAtAngle(MyMath.loopAngle(driveAngle, spinAngle()));
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

    boolean justRestarted;
    void resetDistances()
    {
        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        justRestarted = true;
    }

    private double[] cumulativeDistance = new double[]{0,0};

    private boolean justResetTarget = false;
    ArrayList<Boolean> checkpoints = new ArrayList<>();
    private ArrayList<String> keyList = new ArrayList<>();
    boolean moveOnPath(double[] ... args){
        return moveOnPath(Positioning.Relative, false, args);
    }
    boolean moveOnPath(String key, double[] ... args){
        return moveOnPath(key, Positioning.Relative, false, args);
    }

    boolean moveOnPath(String key, Positioning positioning, boolean continuous, double[] ... args){
        if(keyList.contains(key))
            return true;
        if(moveOnPath(positioning, continuous, args)){
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

    /**
     *@param positioning is the type of positioning system.
     *      *               Absolute is relative the origin point.
     *      *               Relative is relative to robot's last position.
     * @param continuous means that the function will never return true and restart;
     *         it will continue to correct for error.
     *         Use true when you continually test moving to different points.
     *         Use false when the code changes to a new task after completion.
     * @param args is a list of a set of points. Each set looks like this: {3,4}
     *             and is encapsulated in a double[].
     *             The double[]s are added with commas to separate them.
     * @return if all checkpoints have been completed.
     */
    boolean moveOnPath(Positioning positioning, boolean continuous, double[] ... args)
    {
        if(checkpoints.isEmpty()){
            for (double[] ignored : args) checkpoints.add(false);
            justResetTarget = true;
        }

        int c = 0;
        for(boolean b: checkpoints)
            if(b)
                c++;

        telemetry.addData("checkpoints count", c);
        if(c == checkpoints.size())
        {
            checkpoints.clear();
            return true;
        }

        double targetAngle = forward;
        double currentAngle = spinAngle();
        switch (args[c].length)
        {
            case 1:
                targetAngle = forward + args[c][0];

                if(Math.abs(MyMath.loopAngle(targetAngle, currentAngle)) < MyMath.radians(2)) {
                    if(!continuous || c != checkpoints.size() - 1){
                        stop();
                        forward += args[c][0];
                        sumSpinError = 0;
                        checkpoints.set(c, true);
                        cumulativeDistance = new double[]{0,0};
                        resetDistances();
                        justResetTarget = true;
                        break;
                    }
                }
                rotate(angularVelocityNeededToFace(targetAngle));
                justResetTarget = false;
                break;
            case 3:
                targetAngle = forward + args[c][2];

            case 2:
                double[] point = args[c];
                double spin = angularVelocityNeededToFace(targetAngle);

                double deltaX = 0;
                double deltaY = 0;

                switch (positioning)
                {
                    case Absolute:
                        double trueDeltaX = point[0] - trueX;
                        double trueDeltaY = point[1] - trueY;
                        deltaX =  trueDeltaX * Math.cos(-currentAngle) - trueDeltaY * Math.sin(-currentAngle);
                        deltaY =  trueDeltaX * Math.sin(-currentAngle) + trueDeltaY * Math.cos(-currentAngle);
                        break;
                    case Relative:
                        deltaX = point[0] + cumulativeDistance[0] - xDist();
                        deltaY = point[1] + cumulativeDistance[1] - yDist();
                }

                telemetry.addData("deltaX", deltaX);
                telemetry.addData("deltaY", deltaY);

                double r = Math.hypot(deltaX, deltaY);

                double[] drive = move(deltaX, deltaY);



                double driveX = drive[0];
                double driveY = drive[1];

                if(r <= .75 || Math.hypot(driveX, driveY) == 0 //happens when theta changes rapidly
                    && !(continuous && c == checkpoints.size() - 1 )){ //Don't move on if continuous
                        stop();
                        if(args[c].length == 3) {
                            forward += args[c][2];
                            sumSpinError = 0;
                        }
                        this.checkpoints.set(c, true);
                        cumulativeDistance[0] += args[c][0];
                        cumulativeDistance[1] += args[c][1];
                        drdtArray = new ArrayList<>();
                        dThdtArray = new ArrayList<>();
                        justResetTarget = true;
                        break;
                }
                else {
                    //smooth the driving when revving to a high speed, then reset the average to 0.
                    if(Math.hypot(drive[0],drive[1]) > .3)
                        drive(0,false,SmoothingType.Linear, 1,
                                false, false,drive[0], drive[1], spin);
                    else
                        drive(0, false, drive[0], drive[1], spin);
                }
                justResetTarget = false;
                break;
        }
        return false;
    }



    double angularVelocityNeededToFaceForward(){
        return angularVelocityNeededToFace(forward);
    }

    private double sumSpinError = 0;
    private double lastSpinError = 0;
    private double lastSpinTime = 0;

    double sP = .16; // .16 per radian
    double sI = 8; //Time to correct past error
    double sD = .5; //fully account for this much time in the future at current error decreasing rate

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
    void resetForward()
    {
        forward = spinAngle();
        sumSpinError = 0;
    }


    private double lastR = 0;
    private double lastTheta = 0;
    private double lastT = 0;
    double mP = .02; //power per inch
    double mD = 0.5;  //fully account for this much time in the future at current error decreasing rate
    double tD = 0.08;

    private ArrayList<Double> drdtArray = new ArrayList<>();
    private ArrayList<Double> dThdtArray = new ArrayList<>();

    double[] move(double deltaX, double deltaY)
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
        double dTheta = MyMath.loopAngle(theta, lastTheta);
        if(justResetTarget)
            dTheta = 0;
        double drdt = dr / dt;
        drdtArray.add(drdt);

        if(drdtArray.size() > 5) //keep the size to 5
            drdtArray.remove(0);


        double dThdt =  dTheta / dt;
        dThdtArray.add(dThdt);

        if(dThdtArray.size() > 11) //keep the size to 11
            dThdtArray.remove(0);


        telemetry.addData("theta", theta);
        telemetry.addData("dTheta", dTheta);

        //negative because our target is zero
        double tangentPower = -Range.clip(mP * (tD * MyMath.ave(dThdtArray) * r), -.1, .1);
        //dTheta*r / dt is in inches / sec
        //therefore tD is in seconds

        telemetry.addData("dtheta/dt / r", dTheta/dr/r);
        if(dr != 0 && Math.abs(dTheta / dr / r ) > 2){ //2 rad per inch^2
            return new double[]{0,0};
        }

        double power = mP * (r +  mD * MyMath.ave(drdtArray));
        telemetry.addData("moveD term", mD * MyMath.ave(drdtArray));
        telemetry.addData("power", power);
        telemetry.addData("moveD / power",mD * MyMath.ave(drdtArray) / power);

        telemetry.addData("moveT term", tangentPower);
        if(power > .5) power = .5;
        //Don't worry, it's smoothed

        lastR = r;
        lastT = t;
        lastTheta = theta;

        return new double[]{Math.cos(theta) * power - Math.sin(theta) * tangentPower,
                            Math.sin(theta) * power + Math.cos(theta) * tangentPower};
    }

    

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
     * X is in the direction of 0, or right.
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

        if(justRestarted){
            justRestarted = false;
            dX = 0;
            dY = 0;
        }

        double xPrime = dX * Math.cos(spin) - dY * Math.sin(spin);
        double yPrime = dX * Math.sin(spin) + dY * Math.cos(spin);

        trueX += xPrime;
        trueY += yPrime;

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
        FtcRobotControllerActivity.moveRobot(trueX * 4.5, -trueY * 5.5, MyMath.degrees(spinAngle()));
    }
}