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
    var rightForearmWidth=$("#cam1").outerWidth();
    var rightForearmHeight=rightForearmWidth;


    var elmntmjpegHead = document.getElementById("camera2");
    var headWidth=$("#cam2").outerWidth();
    var headHeight=headWidth;

    var elmntmjpegLeftForearm = document.getElementById("camera3");
    var leftForearmWidth=$("#cam3").outerWidth();
    var leftForearmHeight=elmntmjpegLeftForearm.clientHeight;


  // Create the right forearm viewer.
    var forearmRViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width : rightForearmWidth,
        height : rightForearmHeight,
        topic : 'rviz1/camera1/image'
    });

    // Create the head viewer.
    var camera3er = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width : headWidth,
        height : headHeight,
        topic : 'rviz1/camera2/image'
    });

    // Create the left forearm viewer.
    var forearmLViewer = new MJPEGCANVAS.Viewer({
        divID : 'camera3',
        host : 'localhost',
        width : leftForearmWidth,
        height : leftForearmWidth,
        topic : '/wide_stereo/right/image_raw'
    });

var camera1 = document.getElementById("camera1");
var camera2 = document.getElementById("camera2");
var camera3 = document.getElementById("camera3");

var titleWeb = document.getElementById("title");
var titleWebH = titleWeb.offsetHeight;

var btnTop = document.getElementById("top");
var btnTopH = btnTop.offsetHeight;

var btnUpH = camera1.offsetHeight + titleWebH + btnTopH;
var upBTN = document.createElement("button");
upBTN.innerHTML="<img src=\"img/upArrow.png\">";
upBTN.id='up';
upBTN.title='\u2191Up\u2191';
var body = document.getElementsByTagName("body")[0];
body.appendChild(upBTN);

var downBTN = document.createElement("button");
downBTN.innerHTML = "<img src=\"img/downArrow.png\">";
downBTN.id='down';
downBTN.title='\u2193Down\u2193';
downBTN.className = "button";
downBTN.style.left=(camera1.offsetWidth - 95)/2 + "px";
downBTN.style.top= (btnUpH - titleWebH/1.5)/1 + "px";
body.appendChild(downBTN);

var leftBTN = document.createElement("button");
leftBTN.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
leftBTN.id='left';
leftBTN.title='\u2190Left\u2190';
leftBTN.className = "button";
leftBTN.style.left=0 + "px";
leftBTN.style.top= (btnUpH)/2 + "px";
leftBTN.style.width=55 + "px";
leftBTN.style.height=80 + "px";
body.appendChild(leftBTN);

var rotateRightBTN = document.createElement("button");
rotateRightBTN.innerHTML = "<img src=\"img/rotateClockwiseRight.png\">";
rotateRightBTN.id='rotateRight';
rotateRightBTN.title='\u21BBRotate Right\u21BB';
rotateRightBTN.className = "button";
rotateRightBTN.style.left=55 + "px";
rotateRightBTN.style.top= (btnUpH)/2 + "px";
rotateRightBTN.style.height=80 + "px";
body.appendChild(rotateRightBTN);

var rightBTN = document.createElement("button");
rightBTN.innerHTML = "<img src=\"img/greenRightArrow.png\">";
rightBTN.id='right';
rightBTN.title='\u2192Right\u2192';
rightBTN.className = "button";
rightBTN.style.width=55 + "px";
rightBTN.style.height=80 + "px";
rightBTN.style.left= camera1.offsetWidth - titleWebH - 55/4+ "px";
rightBTN.style.top= (btnUpH)/2 + "px";
body.appendChild(rightBTN);


var rotateLeftBTN = document.createElement("button");
rotateLeftBTN.innerHTML = "<img src=\"img/rotateAntiClockwiseLeft.png\">";
rotateLeftBTN.id='rotateLeft';
rotateLeftBTN.title='\u21BARotate Left\u21BA';
rotateLeftBTN.className = "button";
rotateLeftBTN.style.width=65 + "px";
rotateLeftBTN.style.height=80 + "px";
rotateLeftBTN.style.left=camera1.offsetWidth - titleWebH - 75+ "px";
rotateLeftBTN.style.top= (btnUpH)/2 + "px";
rotateLeftBTN.style.height=80 + "px";
body.appendChild(rotateLeftBTN);

upBTN.addEventListener ("click", function() {
  alert("Go Up");
});

downBTN.addEventListener ("click", function() {
  alert("Go Down");
});

leftBTN.addEventListener ("click", function() {
  alert("Go Left");
});

rightBTN.addEventListener ("click", function() {
  alert("Go Right");
});

rotateRightBTN.addEventListener ("click", function() {
  alert("Rotate Right");
});

$("#up").hide();
$("#down").hide();
$("#left").hide()
$("#right").hide();
$("#rotateRight").hide()
$("#rotateLeft").hide();

camera1.addEventListener("mouseover", showButtons);
camera1.addEventListener("mouseout", hideButtons);
camera2.addEventListener("mouseover", showButtons);
camera2.addEventListener("mouseout", hideButtons);
camera3.addEventListener("mouseover", showButtons);
camera3.addEventListener("mouseout", hideButtons);

function showButtons() {

switch(this.id) {
    case "camera1":
        $("#cam1").css("color", "yellow");

upBTN.style.left=(camera1.offsetWidth - 95)/2 + "px";
upBTN.style.top= (btnUpH)/4 + "px";
        $("#up").show();
        $("#down").show();
        $("#left").show();
        $("#right").show();
        $("#rotateRight").show()
        $("#rotateLeft").show();
        break;
    case "camera2":
        $("#cam2").css("color", "yellow");
             //$("#up").show();
      //  $("#down").show();
      //  $("#left").show();
      //  $("#right").show();
        break;
    case "camera3":
        $("#title3").css("color", "yellow");
        break;
    }
}

function hideButtons() {

switch(this.id) {
    case "camera1":
        $("#title1").css("color", "blue");
        $("#up").hide();
        $("#down").hide();
        $("#left").hide();
        $("#right").hide()
        $("#rotateRight").hide()
        $("#rotateLeft").hide();
        break;
    case "camera2":
        $("#title2").css("color", "blue");
        break;
    case "camera3":
        $("#title3").css("color", "white");
        break;
    }
}

init_flag = false;
};



