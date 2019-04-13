package org.firstinspires.ftc.teamcode;

class DriveValuePacket {
    double rSeconds;
    boolean op;
    DriveEngine.SmoothingType type;
    boolean smoothTheta;
    boolean smoothSpin;
    double[] args;
    double x;
    double y;
    double spin;

    /**
     * @param op means overpowered: When op, one motor is maxed out if the overall power > .9
     * @param smoothSpin: Whether the spin should be smoothed or taken as is
     * @param rSeconds: Time that the power smoothing should take to reach exactly the input values.
     * @param smoothTheta: Whether the angle of power should be smoothed or taken as is
     * @param type of smoothing: Exponential, Linear, or None
     * @param args: X, Y, and Spin; 1, 2, or 3 are fine.
     */
    DriveValuePacket(double rSeconds, boolean op, DriveEngine.SmoothingType type,
                     boolean smoothTheta, boolean smoothSpin, double... args)
    {
        this.op = op;
        this.smoothSpin = smoothSpin;
        this.rSeconds = rSeconds;
        this.smoothTheta = smoothTheta;
        this.type = type;
        this.args = args;

        switch (args.length)    //assign x, y and spin
        {
            case 3:
                spin = args[2];  //x,y,spin
            case 2:
                x = args[0];     //x,y
                y = args[1];
                break;
            case 1:
                spin = args[0];  //spin
                break;
        }
    }
}
