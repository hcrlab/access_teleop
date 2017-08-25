/**
 * Created by timadamson on 8/23/17.
 */

// Adds jQuery to the html
var script = document.createElement('script');
script.src = 'http://code.jquery.com/jquery-1.11.0.min.js';
script.type = 'text/javascript';
document.getElementsByTagName('head')[0].appendChild(script);


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

