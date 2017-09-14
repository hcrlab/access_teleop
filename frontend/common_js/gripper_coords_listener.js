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
        var circle;
        if(message.camera_name ==="camera1") {
            circle = document.querySelector("#camera1 circle");
            circle.cx.baseVal.value = message.pixel_x;
            circle.cy.baseVal.value = message.pixel_y + 55;
            self.cam1X = message.pixel_x;
            self.cam1Y = message.pixel_y;
        }
        else if(message.camera_name ==="camera2"){
            circle = document.querySelector("#camera2 circle");
            circle.cx.baseVal.value = message.pixel_x;
            circle.cy.baseVal.value = app.cameraHeight - message.pixel_y;
            self.cam2X = message.pixel_x;
            self.cam2Y = message.pixel_y;
        }
         //console.log('Received message on ' + coords.name + ': ' + message.camera_name);
    });
};