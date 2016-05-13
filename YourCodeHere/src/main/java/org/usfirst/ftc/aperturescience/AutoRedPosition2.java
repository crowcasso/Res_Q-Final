package org.usfirst.ftc.aperturescience;

/**
 * AutoRedPosition2 (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="RED Beach Bum Jimmy", group="Red")
public class AutoRedPosition2 extends AutoCommon {

    // how far to stay away from the wall
    private final double THE_DISTANCE = 21;

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // pull out the tapes to make room for the arm
        setTapes();

        // pause for 3 seconds
        Thread.sleep(6000);

        // move the robot toward the mountain
        driveBack(.2, 6);
        Thread.sleep(500);
        turnGyroSlow(-90);
        Thread.sleep(500);
        driveBack(.2, 35);
        Thread.sleep(500);
        turnGyroSlow(55);
        Thread.sleep(500);

        // drive back 90 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.6, 90);
        Thread.sleep(200);  // small pause

        if (foundWhite) {
            // yeah! found the white line

            // pause to let the gyro settle
            Thread.sleep(500);

            // turn left 49 degrees
            turnGyro(-47);
            Thread.sleep(200);

            // drive back to wall using the ultrasonic
            driveToDistance(.2, 6, THE_DISTANCE);

        } else {
            // did not find white line -- let's try to correct

            // drive forward 10 inches
            drive(.3, 10);
            Thread.sleep(500);

            // turn left 56 degrees
            turnGyro(-56);
            Thread.sleep(500);

            // drive back to wall using ultrasonic
            driveToDistance(.2, 6, THE_DISTANCE);
        }

        // bring the arm up
        autoArmUp();

        dropBucket();
        Thread.sleep(1000);

        // bring the arm back into the robot to get ready for TeleOp
        autoArmDown();
    }
}