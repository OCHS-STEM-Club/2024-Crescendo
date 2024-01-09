// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {
        public static double kMaxSpeed = 3; //meters per second
        public static double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second


        //Chassis Config
        public static final double kTrackWidth = Units.inchesToMeters(25.5);
        //Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        //Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        
    }

    public static final class ModuleConstants {

    }

    public static final class AutoConstants {

    }

    public static final class NeoMotorConstants {

    }
}


