package com.epra.epralib.ftclib.location.imu;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/// An [IMUWrapper] for an [IMU].
/// @author jfishCoder
/// @author Striker-909
/// @since 1.7.0
public class IMUFrame implements IMUWrapper {

    private final IMU imu;

    public IMUFrame(IMU imu) {
        this.imu = imu;
    }

    /// Returns the yaw of the imu as an [Angle].
    /// @return the yaw of the imu
    @Override
    public Angle getYaw() {
        return Angle.degree(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
    /// Returns the roll of the imu as an [Angle].
    /// @return the roll of the imu
    @Override
    public Angle getRoll() {
        return Angle.degree(imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
    }
    /// Returns the pitch of the imu as an [Angle].
    /// @return the pitch of the imu
    @Override
    public Angle getPitch() {
        return Angle.degree(imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
    }
    /// Returns the yaw angular velocity of the imu as an [Angle].
    /// @return the yaw angular velocity of the imu
    @Override
    public Angle getYawVelocity() {
        return Angle.degree(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
    }
    /// Returns the roll angular velocity of the imu as an [Angle].
    /// @return the roll angular velocity of the imu
    @Override
    public Angle getRollVelocity() {
        return Angle.degree(imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate);
    }
    /// Returns the pitch angular velocity of the imu as an [Angle].
    /// @return the pitch angular velocity of the imu
    @Override
    public Angle getPitchVelocity() {
        return Angle.degree(imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
    }
}
