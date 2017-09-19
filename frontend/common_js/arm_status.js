/**
 * Created by timadamson on 9/14/17.
 */

// Subscribing to a Topic
// ----------------------

ArmStatus = function(ros) {

    var self = this;

    var arm_status = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/arm_status',
        messageType: 'std_msgs/String'
    });

    arm_status.subscribe(function (message) {
        app.handleStatus(message)
    });
};