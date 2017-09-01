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

    this.arm = new Arm(this.ros);
    this.gripper = new Gripper(this.ros);
    //self.head = new Head(ros);


    // Set up the gripper event handlers
    // Calls itself after definition
    this.initRightClickGripper = function () {
        var arm_div = document.querySelectorAll('.js_arm_div');
        arm_div.forEach(function(element){
            element.addEventListener('contextmenu', function(ev){
                ev.preventDefault();
                if(self.gripper.getCurrentPosition() == self.gripper.PositionEnum.CLOSED ||
                    self.gripper.getCurrentPosition() == self.gripper.PositionEnum.PARTLY_CLOSED) {
                    self.gripper.open();
                }
                else {
                    self.gripper.close();
                }
               return false;
            }, false);
        });
    };



    // Adds 3 canvas image streams
    // --------------------------------------------------------------------------------

    // Dynamic Canvas Sizes
    var elmntmjpegRightForearm = document.getElementById("camera1");
    var rightForearmWidth=elmntmjpegRightForearm.clientWidth;
    var rightForearmHeight=elmntmjpegRightForearm.clientHeight;


    var elmntmjpegHead = document.getElementById("camera2");
    var headWidth=elmntmjpegHead.clientWidth;
    var headHeight=elmntmjpegHead.clientHeight;
    var camera2Height = "480";
    var camera2Width = "640";

    var elmntmjpegLeftForearm = document.getElementById("mjpegLeftForearm");
    var leftForearmWidth=elmntmjpegLeftForearm.clientWidth;
    var leftForearmHeight=elmntmjpegLeftForearm.clientHeight;

    // Create the right forearm viewer.
    var forearmRViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width : rightForearmWidth,
        height : rightForearmHeight,
        topic : '/rviz1/camera1/image'
    });

    // Create the head viewer.
    var headViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width : headWidth,
        height : headHeight,
        topic : '/rviz1/camera2/image'
    });

    // Create the left forearm viewer.
    var forearmLViewer = new MJPEGCANVAS.Viewer({
        divID : 'mjpegLeftForearm',
        host : 'localhost',
        width : leftForearmWidth,
        height : leftForearmWidth,
        topic : '/head_camera/rgb/image_raw'
    });

    // ------------------------------------------------------------------------------------------

    init_flag = false;
};



