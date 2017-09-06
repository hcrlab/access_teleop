/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */

App = function () {

    // Set self to be this so that you can add variables to this inside a callback
    var self = this;

    // Set up ros
    this.ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });


    this.ros.on('error', function(error) {
        console.log('Error connecting to websocket server.');
    });

    self.arm = new Arm(this.ros);
    //self.head = new Head(ros);
    //self.gripper = new Gripper(ros);




    // Adds 3 canvas image streams
    // --------------------------------------------------------------------------------

    var cameraWidth = "640";
    var cameraHeight = "480";

    // Dynamic Canvas Sizes
    var elmntmjpegRightForearm = document.getElementById("camera1");
    var camera1Width=cameraWidth;
    var camera1Height=cameraHeight;


    var elmntmjpegHead = document.getElementById("camera2");
    var camera2Width = cameraWidth;
    var camera2Height = cameraHeight;

    var elmntmjpegLeftForearm = document.getElementById("camera3");
    var leftForearmWidth=elmntmjpegLeftForearm.clientWidth;
    var leftForearmHeight=elmntmjpegLeftForearm.clientHeight;

    // Create the right forearm viewer.
    var forearmRViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width : camera1Width,
        height : camera1Height,
        topic : '/rviz1/camera1/image'
    });

    // Create the head viewer.
    var headViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width : camera2Width,
        height : camera2Height,
        topic : '/rviz1/camera2/image'
    });

    // Create the left forearm viewer.
    var forearmLViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera3',
        host : 'localhost',
        width : leftForearmWidth,
        height : leftForearmWidth,
        topic : '/head_camera/rgb/image_raw'
    });

    // ------------------------------------------------------------------------------------------

    init_flag = false;
};



