/**
 * Created by timadamson on 9/13/17.
 */

// Subscribing to a Topic
    // ----------------------

CoordsListener = function(ros) {

    this.cam1X = 0;
    this.cam1Y = 0;
    this.cam2X = 0;
    this.cam2Y = 0;

    var self = this;

    var coords = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/gripper_pixels',
        messageType: 'access_teleop_msgs/PX'
    });

    coords.subscribe(function (message) {
        if(message.camera_name === "camera1") {
            self.cam1X = message.pixel_x;
            self.cam1Y = message.pixel_y;
        }
        if(message.camera_name === "camera2") {
            self.cam2X = message.pixel_x;
            self.cam2Y = app.cameraHeight - message.pixel_y;
        }
        app.handleGripperCoords(message);
    });
};