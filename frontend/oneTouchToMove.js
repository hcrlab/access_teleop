/**
 * Created by timadamson on 8/23/17.
 */

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();

    this.app.armStatus = new ArmStatus(this.app.ros);

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
                            var x_pixel = parseInt((self.app.backendCameraWidth / self.app.cameraWidth) * e.offsetX);
                            var y_pixel = parseInt((self.app.backendCameraHeight / self.app.cameraHeight) * e.offsetY);
                            //console.log(elementId + " " + x_pixel + " " + y_pixel);
                            self.app.arm.moveArmByAbsolute(x_pixel, y_pixel, elementId);
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

