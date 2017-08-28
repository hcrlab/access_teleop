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

var elmnt1 = document.getElementById("title");

var elmnt = document.getElementById("camera1");

var upBTN = document.createElement("button");
upBTN.innerHTML = "\u2191Up\u2191";
upBTN.id='up';
upBTN.style.left=(elmnt.offsetWidth- 32)/2 + "px";
upBTN.style.top= (elmnt.offsetHeight- 32)/2 + "px";
var body = document.getElementsByTagName("body")[0];
body.appendChild(upBTN);

var downBTN = document.createElement("button");
downBTN.innerHTML = "\u2191Down\u2191";
downBTN.id='down';
downBTN.className = "button";
downBTN.style.left=(elmnt.offsetWidth- 32)/2 + "px";
downBTN.style.top= (elmnt.offsetHeight- 32) + "px";
var body = document.getElementsByTagName("body")[0];
body.appendChild(downBTN);

upBTN.addEventListener ("click", function() {
  alert("Go Up");
});

downBTN.addEventListener ("click", function() {
  alert("Go Down");
});

init_flag = false;
};



