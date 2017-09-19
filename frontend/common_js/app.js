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

    this.ros.on('close', function (error) {
       console.error('We lost connection with ROS. All is lost');
       document.body.innerHTML = "The connection with ROS is broken. Please reconnect";
    });

    this.arm = new Arm(this.ros);
    this.gripper = new Gripper(this.ros);
    this.cloudFreezer = new CloudFreezer(this.ros);
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

    this.addCloudFreezer = function(){
        var feedback = document.querySelector("#feedback");

        var freezeButton = document.createElement("button");
        freezeButton.innerHTML = "Freeze Point Cloud";
        freezeButton.onclick = this.cloudFreezer.freezeCloud;

        var unfreezeButton = document.createElement("button");
        unfreezeButton.innerHTML = "Real Time Point Cloud";
        unfreezeButton.onclick = this.cloudFreezer.unfreezeCloud;

        feedback.appendChild(freezeButton);
        feedback.appendChild(unfreezeButton);
    };



    // Adds 3 canvas image streams
    // --------------------------------------------------------------------------------
    // Dynamic Canvas Sizes
    var camCanvas = document.getElementById("cam1");

    this.cameraWidth = "640";
    this.cameraHeight = "480";

    this.backendCameraWidth = "640";
    this.backendCameraHeight = "480";

    // Dynamic Canvas Sizes
    var rightForearmWidth=this.cameraWidth;
    var rightForearmHeight=this.cameraHeight;


    var headWidth=this.cameraWidth;
    var headHeight=this.cameraHeight;


    var elmntmjpegLeftForearm = document.getElementById("camera3");
    var leftForearmWidth=elmntmjpegLeftForearm.clientWidth;
    var leftForearmHeight=leftForearmWidth;

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
        divID : 'camera3',
        host : 'localhost',
        width : leftForearmWidth,
        height : leftForearmHeight,
        topic : '/head_camera/rgb/image_raw'
    });

    // ------------------------------------------------------------------------------------------

    init_flag = false;
};



