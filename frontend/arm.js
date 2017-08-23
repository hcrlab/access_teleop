/**
 * Created by timadamson on 8/22/17.
 */

var prevX;
var prevY;

Arm = function (ros) {
    var arm_div = document.querySelectorAll('.js_arm_div');

    var that = this;

    // Set up the arm publisher
    var delta = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/delta',
        messageType: 'access_teleop_msgs/DeltaPX'
    });

    var absolute = new ROSLIB.Topic({
       ros: ros,
       name: '/access_teleop/absolute',
       messageType: 'access_teleop_msgs/PX'
    });

    var moveArmByDelta = function (deltaX, deltaY, cameraName) {
        var deltaPX = new ROSLIB.Message({
            camera_name: cameraName,
            delta_x: deltaX,
            delta_y: deltaY
        });
        delta.publish(deltaPX);
    }

    var moveArmByAbsolute = function (absX, absY, cameraName) {
        var absolutePX = new ROSLIB.Message({
            camera_name: cameraName,
            pixel_x: absX,
            pixel_y: absY
        });
        absolute.publish(absolutePX);
    }

    arm_div.forEach(function(element)
    {
        element.onmousedown = function (e) {
            e = e || window.event;
            var elementId = (e.target || e.srcElement).parentElement.id;
            console.log(elementId);
            console.log("offsetX :" + e.offsetX + " offsetY : " + e.offsetY);
            if (prevX && prevY) {
                moveArmByDelta(e.offsetX - prevX, e.offsetY - prevY, elementId);
                prevX = 0;
                prevY = 0;
            } else {
                prevX = e.offsetX;
                prevY = e.offsetY;
            }
        }
    });


}