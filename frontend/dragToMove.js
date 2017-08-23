/**
 * Created by timadamson on 8/23/17.
 */

// Public variables
    var downX;
    var downY;

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

var cmdVelTopic;
var twist;

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    app.ros.on('connection', function () {
        console.log("We are connected!");

        arm_div.forEach(function(element)
        {
            element.onmousedown = function (e) {
                e = e || window.event;
                var elementId = (e.target || e.srcElement).parentElement.id;
                console.log(elementId);
                downX = e.offsetX;
                downY = e.offsetY;
            };

            element.onmouseup = function (e) {
                e = e || window.event;
                var elementId = (e.target || e.srcElement).parentElement.id;
                console.log(elementId);
                console.log("offsetX :" + e.offsetX + " offsetY : " + e.offsetY);
                self.app.arm.moveArmByDelta(e.offsetX - downX, e.offsetY - downY, elementId);
            };

        });

    });

// Publishing a Topic
// ------------------
     cmdVelTopic = new ROSLIB.Topic({
        ros : app.ros,
        name : '/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
     twist = new ROSLIB.Message({
        linear : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        },
        angular : {
            x : 0.0,
            y : 0.0,
            z : 0.0
        }
    });

}





/* This function:
 - retrieves numeric values from the text boxes
 - assigns these values to the appropriate values in the twist message
 - publishes the message to the cmd_vel topic.
 */
function pubMessage() {
    /**
     Set the appropriate values on the twist message object according to values in text boxes
     It seems that turtlesim only uses the x property of the linear object
     and the z property of the angular object
     **/
    var linearX = 0.0;
    var angularZ = 0.0;

    // get values from text input fields. Note for simplicity we are not validating.
    linearX = 0 + Number(document.getElementById('linearXText').value);
    angularZ = 0 + Number(document.getElementById('angularZText').value);

    // Set the appropriate values on the message object
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    // Publish the message
    cmdVelTopic.publish(twist);
    //  that.emit('change', twist);
    document.getElementById("demo").innerHTML = "Updated Position";
}