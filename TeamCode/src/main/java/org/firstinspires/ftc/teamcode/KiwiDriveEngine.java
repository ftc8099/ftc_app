package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class KiwiDriveEngine extends OmniWheelDriveEngine
{
    OptimisedRoadRunnerKiwiDrive optimisedKiwiDrive;

    KiwiDriveEngine(HardwareMap hardwareMap, Telemetry telemetry, Sensors sensors)
    {
        super(hardwareMap, telemetry, sensors, 3);
        optimisedKiwiDrive = new OptimisedRoadRunnerKiwiDrive(hardwareMap);
        motors = new ArrayList<>(optimisedKiwiDrive.motors);
    }

    Trajectory currentTrajectory;

    @Override
    boolean moveOnPath(Positioning positioning, double[]... args) {
        boolean newPath = resetCheckpointsIfNeeded(args);


        if(newPath)
        {
            BaseTrajectoryBuilder builder = optimisedKiwiDrive.trajectoryBuilder();

            for (double[] checkpoint:args) {

                double currentAngle = getCurrentAngle();

                switch (checkpoint.length)
                {
                    case 1:
                        builder.splineTo(new Pose2d(lastCheckpoint[0], lastCheckpoint[1], checkpoint[0]), new SplineInterpolator(forward,checkpoint[0]));
                        break;
                    case 4:

                    case 3:

                    case 2:

                        double[] distances = distancesToTarget(positioning, checkpoint, currentAngle);

                        double deltaX = distances[0];
                        double deltaY = distances[1];

                        builder.splineTo(new Pose2d(deltaX, deltaY));
                        break;
                }

                lastCheckpoint = checkpoint;

            }
            currentTrajectory = builder.build();
        }

        return optimisedKiwiDrive.followTrajectorySync(currentTrajectory);
    }

}
