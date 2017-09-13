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


    // A function for publishing to /access_teleop/delta
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

    this.moveAndOrient = function (absX, absY, theta, cameraName) {
        var absoluteAndTheta = new ROSLIB.Message({
            camera_name: cameraName,
            pixel_x: absX,
            pixel_y: absY,
            theta: theta
        });
        moveAndOrient.publish(absoluteAndTheta);
    };

};