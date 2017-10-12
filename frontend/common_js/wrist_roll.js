/**
 * Created by timadamson on 9/15/17.
 */


WristRoller = function (ros) {

    // Set up the freeze publishers
    var wristRoller = new ROSLIB.Topic({
        ros: ros,
        name: 'access_teleop_msgs/Theta',
        messageType: '/access_teleop/wrist_roll'
    });

    // A function for publishing 'theta' to access_teleop_msgs/Theta
    // No camera name required
    this.rotate = function (theta){
        var rotateTheta = new ROSLIB.Message({
            camera_name: "",
            theta: theta
        });
        wristRoller.publish(rotateTheta);
    };

};