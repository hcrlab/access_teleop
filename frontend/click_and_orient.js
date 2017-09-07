/**
 * Created by timadamson on 9/6/17.
 */

var downX;
var downY;

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    app.ros.on('connection', function () {
        console.log("We are connected!");

        app.initRightClickGripper(); // This adds the right click gripper listener

        arm_div.forEach(function(element)
        {
            element.onmousedown = function (e) {
                e = e || window.event;
                if(e.which == 1) { //This will only be true on a left click
                    var elementId = (e.target || e.srcElement).parentElement.id;
                    console.log(elementId);
                    downX = e.offsetX;
                    downY = e.offsetY;
                }
            };

            element.onmouseup = function (e) {
                e = e || window.event;
                if(e.which == 1) { //This will only be true on a left click
                    var elementId = (e.target || e.srcElement).parentElement.id;
                    console.log(elementId);
                    console.log("Mouse down happend at x:" + downX + " , y:" + downY);
                    console.log("offsetX :" + e.offsetX + " offsetY : " + e.offsetY);
                    var xDiff = e.offsetX - downX;
                    var yDiff = e.offsetY - downY;
                    var angle = -Math.atan2(yDiff, xDiff);
                    self.app.arm.moveArmByDelta(xDiff, yDiff, elementId);
                    console.log("The angle we got is " + angle);
                }
            };
        });

    });


}