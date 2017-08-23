/**
 * Created by timadamson on 8/22/17.
 */

/**
 * Setup all visualization elements when the page is loaded.
 */

var script = document.createElement('script');
script.src = 'http://code.jquery.com/jquery-1.11.0.min.js';
script.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script);


function showCoords(evt){
    if(prevX && prevY){
        xMid = document.getElementById("camera1").clientWidth / 2;
        yMid = document.getElementById("camera2").clientHeight / 2;
        console.log("The delta x is " + (evt.offsetX - prevX) + " and the delta y is " + (evt.offsetY - prevY));
        console.log("The x coord is " + (evt.offsetX - xMid) + " and the y coord is " + (evt.offsetY - yMid));
    }
    prevX = evt.offsetX;
    prevY = evt.offsetY;
}

$(document).ready(function () {
    $('body').click(function (ev) {
        mouseX = ev.pageX;
        mouseY = ev.pageY
        var color = '#1daeae';
        var size = '2px';
        $("body").append($('<div></div>')
            .css('position', 'absolute')
            .css('top', mouseY + 'px')
            .css('left', mouseX + 'px')
            .css('width', size)
            .css('height', size)
            .css('background-color', color));
    });
});

App = function () {

    var that = this;

    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    ros.on('connection', function () {
        that.arm = new Arm(ros);
        //that.head = new Head(ros);
    })

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

    init_flag = false;
};
function init() {
    var app = new App();
}


