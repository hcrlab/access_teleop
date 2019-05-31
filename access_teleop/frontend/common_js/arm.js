/**
 * Created by timadamson on 8/22/17.
 */

Arm = function (ros) {

    // Set up the arm publishers
    var delta = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/delta',
        messageType: 'access_teleop_msgs/DeltaPX'
    });

    var absolute = new ROSLIB.Topic({
       ros: ros,
       name: '/access_teleop/absolute',
       messageType: 'access_teleop_msgs/PX'
    });

    var moveAndOrient = new ROSLIB.Topic({
       ros: ros,
       name: '/access_teleop/move_and_orient',
       messageType: 'access_teleop_msgs/PXAndTheta'
    });

    var orient = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/orient',
        messageType: 'access_teleop_msgs/Theta'
    });

    this.adv = function() {
        delta.advertise();
        absolute.advertise();
        moveAndOrient.advertise();
        orient.advertise();
    };

    this.unadv = function() {
        delta.unadvertise();
        absolute.unadvertise();
        moveAndOrient.unadvertise();
        orient.unadvertise();
    };

    // A function for publishing to /access_teleop/delta
    // Moves the end effector some delta x and delta y, which is the difference between the current x and y and the desired
    // x and y. The cameraName is needed so that the backend can determine which world coordinates the x and y
    // correlate with. The function simply creates a ROS message with the x, y, and cameraName passed in and then sends
    // that message to the ROS backend.
    this.moveArmByDelta = function (deltaX, deltaY, cameraName) {
        if(cameraName) {
            var deltaPX = new ROSLIB.Message({
                camera_name: cameraName,
                delta_x: parseInt(deltaX),
                delta_y: parseInt(deltaY)
            });
            delta.publish(deltaPX);
        }
    };

    // A function for publishing to /access_teleop/absolute
    // Moves the end effector to an absolute point x, y. Is agnostic to where the end effector is currently at. The
    // cameraName is needed so that the backend can determine which world coordinates the x and y correlate with. The
    // function simply creates a ROS message with the x, y, and cameraName passed in and then sends that message to
    // the ROS backend.
    this.moveArmByAbsolute = function (absX, absY, cameraName) {
        if(cameraName) {
            var absolutePX = new ROSLIB.Message({
                camera_name: cameraName,
                pixel_x: parseInt(absX),
                pixel_y: parseInt(absY)
            });
            absolute.publish(absolutePX);
        }
    };

    // Moves the end effector to an absolute point x, y, and changes the orientation of the end effector. All movement
    // and orientation change is in a 2D plane that is orthogonal to the camera specified by cameraName.
    this.moveAndOrient = function (absX, absY, theta, cameraName) {
        var absoluteAndTheta = new ROSLIB.Message({
            camera_name: cameraName,
            pixel_x: parseInt(absX),
            pixel_y: parseInt(absY),
            theta: theta
        });
        moveAndOrient.publish(absoluteAndTheta);
    };

    // Changes the orientation of the end effector to be theta. The orientation change is in a 2D plane that is
    // orthogonal to the camera specified by cameraName.
    this.orientByTheta = function(theta, cameraName) {
        var theta = new ROSLIB.Message({
            camera_name: cameraName,
            theta: theta
        });
        orient.publish(theta)
    };

};