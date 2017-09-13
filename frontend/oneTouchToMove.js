/**
 * Created by timadamson on 8/23/17.
 */

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    app.ros.on('connection', function () {
        console.log("We are connected!");

        app.initRightClickGripper(); // This adds the right click gripper listener

        arm_div.forEach(function(element)
        {
            element.onclick = function (e) {

                arm_div.forEach(function (arm_element) {
                    if (!arm_element.onmousemove) {
                        arm_element.style.cursor = 'crosshair';
                        arm_element.onmousemove = function (e) {
                            e = e || window.event;
                            var elementId = (e.target || e.srcElement).parentElement.id;
                            var canvas = document.querySelector("#" + elementId + " canvas");
                            var x_pixel = (self.app.backendCameraWidth / self.app.cameraWidth) * e.offsetX;
                            console.log(elementId);
                            self.app.arm.moveArmByAbsolute(e.offsetX, e.offsetY, elementId);
                        };
                    }
                    else {
                        arm_element.onmousemove = undefined;
                        arm_element.style.cursor = 'default';
                        console.log("removing onmousemove event handeler");
                    }
                });
            };

        });

    });


}

