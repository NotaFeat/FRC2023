package frc.robot;


public interface IDrive {
    
    public Mode getCurrentDriveMode();

    public void resetGyro();

    // Teleoperated methods use radians
    // Turns the robot by a relative angle
    public void rotateRelative(double angle);

    // Turns the robot to an absolute angle
    public void rotateAbsolute(double angle);

    public void balanceOn();

    public void balanceOff();

    public void driveSpeed(double distanceInches, double speed, double angle);

    public void move();
    /*
     * completionRoutine is called when the current action has been completed
     * Autonomous methods use degrees
     */
    public void cartesianMovement(double xSpeed, double ySpeed, double zRotation, Runnable completionRoutine);
    
    public void driveDistance(double distanceInches, double speed, double angle, Runnable completionRoutine);

    public void driveSpeed(double forwardSpeed, double strafeSpeed, double angle, Runnable completionRoutine);

    public void driveRevolutions(double rotations, double speed, double angle, Runnable completionRoutine);

    public void rotateRelative(double angle, Runnable completionRoutine);

    public void rotateAbsolute(double angle, Runnable completionRoutine);

    /*
     * This is the method used to drive manually during teleoperated mode
     */
    public void driveManual(double forwardSpeed, double strafeSpeed);

    // This turns the robot to an absolute field angle

    public void stop();

    public void moveAuto();

    public void init();

    /*
     * Called periodically to actually execute the driving and rotating set by
     * the driveDistance() and rotateDegrees() methods
     */
    public void periodic();
    
}
