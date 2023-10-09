package frc.robot;

public interface IArm {

    public void init();

    public void stop();

    public void resetPosition();

    public void pickUp();
    
    public void autoShoulder(double shoulderPosition, double speed, boolean shoulder, Runnable completionRoutine);

    public void autoElbow(double elbowPosition, double speed, boolean elbow, Runnable completionRoutine);

    public void autoGrabber(double grabberPosition, double speed, boolean grabber, Runnable completionRoutine);

    public void shoulderUp();

    public void shoulderDown();

    public void elbowUp();

    public void elbowDown();

    public void grabberClose();

    public void grabberOpen();

    public void autoArm(Runnable completionRoutine);

    public void periodic();

    public void midCone();

    public void shelf();

    public void floor();

    public void rest();
}
