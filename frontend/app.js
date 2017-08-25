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

    // Dynamic Canvas Sizes
    var elmntmjpegRightForearm = document.getElementById("camera1");
    var rightForearmWidth=elmntmjpegRightForearm.clientWidth;
    var rightForearmHeight=rightForearmWidth;


    var elmntmjpegHead = document.getElementById("camera2");
    var headWidth=elmntmjpegHead.clientWidth;
    var headHeight=headWidth;
//    var camera2Height = "480";
//    var camera2Width = "640";

    var elmntmjpegLeftForearm = document.getElementById("mjpegLeftForearm");
    var leftForearmWidth=elmntmjpegLeftForearm.clientWidth;
    var leftForearmHeight=elmntmjpegLeftForearm.clientHeight;

    var paper1 = Snap( "#svg1" ); //use element created above


    var x = document.createElement("CANVAS");
    var ctx = x.getContext("2d");
    ctx.fillStyle = "#FF0000";
    ctx.fillRect(0, 0, 150, 100);

    // Create the right forearm viewer.
    var forearmRViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width : rightForearmWidth,
        height : rightForearmHeight,
        topic : '/r_forearm_cam/image_raw',
//        topic : '/rviz1/camera1/image'
        overlay: x
    });

    // Create the head viewer.
    var headViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width : headWidth,
        height : headHeight,
        topic : '/wide_stereo/left/image_raw'
//        topic : '/rviz1/camera2/image'
    });

    // Create the left forearm viewer.
    var forearmLViewer = new MJPEGCANVAS.Viewer({
        divID : 'mjpegLeftForearm',
        host : 'localhost',
        width : leftForearmWidth,
        height : leftForearmWidth,
        topic : '/l_forearm_cam/image_raw'
//        topic : '/head_camera/rgb/image_raw'
    });

    // ------------------------------------------------------------------------------------------

    init_flag = false;
};



