/**
 * Created by timadamson on 8/23/17.
 */

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    app.ros.on('connection', function () {
        console.log("We are connected!");

        arm_div.forEach(function(element)
        {
            element.onmousemove = function (e) {
                e = e || window.event;
                var elementId = (e.target || e.srcElement).parentElement.id;
                console.log(elementId);
                self.app.arm.moveArmByAbsolute(e.offsetX, e.offsetY, elementId);
            };

        });

    });


}

