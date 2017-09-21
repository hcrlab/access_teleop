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
        var x_pixel = parseInt(message.pixel_x * (app.cameraWidth / app.backendCameraWidth));
        var y_pixel = parseInt(message.pixel_y * (app.cameraHeight / app.backendCameraHeight));
        if(message.camera_name ==="camera1") {
            circle = document.querySelector("#camera1 circle");
            circle.cx.baseVal.value = x_pixel;
            circle.cy.baseVal.value = y_pixel;
            self.cam1X = x_pixel;
            self.cam1Y = y_pixel;
        }
        else if(message.camera_name ==="camera2"){
            circle = document.querySelector("#camera2 circle");
            circle.cx.baseVal.value = x_pixel;
            circle.cy.baseVal.value = app.cameraHeight - y_pixel;
            self.cam2X = x_pixel;
            self.cam2Y = app.cameraHeight - y_pixel;
        }
         //console.log('Received message on ' + coords.name + ': ' + message.camera_name);
    });
};