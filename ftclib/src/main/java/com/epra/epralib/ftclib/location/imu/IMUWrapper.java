package com.epra.epralib.ftclib.location.imu;

import com.epra.epralib.ftclib.math.geometry.Angle;

/// A wrapper of an IMU.
///
/// @author jfishCoder
/// @author Striker-909
/// @since 1.7.0
public interface IMUWrapper {
    /// Returns the yaw of the imu as an [Angle].
    /// @return the yaw of the imu
    public Angle getYaw();
    /// Returns the pitch of the imu as an [Angle].
    /// @return the pitch of the imu
    public Angle getPitch();
    /// Returns the roll of the imu as an [Angle].
    /// @return the roll of the imu
    public Angle getRoll();
    /// Returns the yaw angular velocity of the imu as an [Angle].
    /// @return the yaw angular velocity of the imu
    public Angle getYawVelocity();
    /// Returns the pitch angular velocity of the imu as an [Angle].
    /// @return the pitch angular velocity of the imu
    public Angle getPitchVelocity();
    /// Returns the roll angular velocity of the imu as an [Angle].
    /// @return the roll angular velocity of the imu
    public Angle getRollVelocity();

}
