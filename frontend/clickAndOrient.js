/**
 * Created by timadamson on 9/6/17.
 */

var downX;
var downY;

function addLine(x1,y1, x2, y2, camera_name){
    var svg = document.querySelector("#" + camera_name + "  svg");
    var line = document.createElement("line");
    line.setAttribute('x1', x1);
    line.setAttribute('y1', y1);
    line.setAttribute('x2', x2);
    line.setAttribute('y2', y2);
    line.setAttribute('stroke', "red");
    line.setAttribute('stroke-width', "2");
    svg.appendChild(line);
}

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
                    var angle = Math.atan2(e.offsetY - downY, e.offsetX - downX);
                    console.log(angle);
                    self.app.arm.moveAndOrient(e.offsetX, e.offsetY, angle, elementId);
                    addLine(downX, downY, e.offsetX, e.offsetY, elementId);
                    console.log("The angle we got is " + angle);
                }
            };
        });

    });


}