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
                    var cameraDiv = (e.target || e.srcElement).closest("div");
                    if(cameraDiv) {
                        downX = e.offsetX;
                        downY = e.offsetY;
                        cameraDiv.onmousemove = function (moveE) {
                            self.app.moveArrow(downX, downY, moveE.offsetX, moveE.offsetY, cameraDiv.id);
                        };
                        console.log("added onmousemove to " + cameraDiv.id);
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
                        console.log("removed onmousemove from " + elementId);
                        console.log("Mouse down happend at x:" + downX + " , y:" + downY);
                        console.log("offsetX :" + e.offsetX + " offsetY : " + e.offsetY);
                        var angle = Math.atan2(e.offsetY - downY, e.offsetX - downX);
                        console.log(angle);
                        var x_pixel = parseInt(e.offsetX * (self.app.backendCameraWidth / self.app.cameraWidth));
                        var y_pixel = parseInt(e.offsetY * (self.app.backendCameraHeight / self.app.cameraHeight));
                        self.app.arm.moveAndOrient(x_pixel, y_pixel, angle, elementId);
                        console.log("The angle we got is " + angle);
                    }
                }
            };

        });

    });


}