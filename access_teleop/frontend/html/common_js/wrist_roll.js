/**
 * Created by timadamson on 9/15/17.
 */


WristRoller = function (ros) {

    // Set up the freeze publishers
    var wristRoller = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/wrist_roll',
        messageType: 'access_teleop_msgs/Theta'
    });

    this.adv = function() {
        wristRoller.advertise();
    };

    this.unadv = function() {
        wristRoller.unadvertise();
    };

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