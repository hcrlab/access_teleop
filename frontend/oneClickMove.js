/**
 * Created by timadamson on 8/23/17.
 */

// Adds jQuery to the html
var script = document.createElement('script');
script.src = 'http://code.jquery.com/jquery-1.11.0.min.js';
script.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script);

// Prints to the console the coords of the user click
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

// Adds a colored div when body is clicked
$(document).ready(function () {
    $('body').click(function (ev) {
        mouseX = ev.pageX;
        mouseY = ev.pageY;
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

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();

    app.ros.on('connection', function () {
        console.log("We are connected!");

    });


}

