/**
 * Created by timadamson on 8/23/17.
 */

// Public variables
var downX;
var downY;
var cmdVelTopic;
var twist;

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    this.app.ros.on('connection', function () {
        console.log("We are connected!");

        self.app.initRightClickGripper(); // This adds the right click gripper listener
        self.app.addCloudFreezer();

        arm_div.forEach(function(element)
        {
            element.onmousedown = function (e) {
                e = e || window.event;
                if(e.which == 1) { //This will only be true on a left click
                    var cameraDiv = (e.target || e.srcElement).closest("div");
                    if(cameraDiv) {
                        downX = e.offsetX;
                        downY = e.offsetY;
                        cameraDiv.onmousemove = function (moveE) {
                            self.app.moveLine(downX, downY, moveE.offsetX, moveE.offsetY, cameraDiv.id);
                        };
                    }
                }
            };

            element.onmouseup = function (e) {
                e = e || window.event;
                if(e.which == 1) { //This will only be true on a left click
                    var cameraDiv = (e.target || e.srcElement).closest("div");
                    if(cameraDiv) {
                        var elementId = cameraDiv.id;
                        cameraDiv.onmousemove = undefined;
                        console.log("mouse move eliminated from " + elementId);
                        console.log("offsetX :" + e.offsetX + " offsetY : " + e.offsetY);
                        var x_pixel = (self.app.backendCameraWidth / self.app.cameraWidth) * (e.offsetX - downX);
                        var y_pixel = (self.app.backendCameraHeight / self.app.cameraHeight) * (e.offsetY - downY);
                        self.app.arm.moveArmByDelta(x_pixel, y_pixel, elementId);
                    }
                }
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
    linearX = Number(document.getElementById('linearXText').value);
    angularZ = Number(document.getElementById('angularZText').value);

    // Set the appropriate values on the message object
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    // Publish the message
    cmdVelTopic.publish(twist);
    //  that.emit('change', twist);
    document.getElementById("demo").innerHTML = "Updated Position";
}