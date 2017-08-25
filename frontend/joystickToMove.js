/**
 * Created by timadamson on 8/24/17.
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

    app.ros.on('connection', function () {
        console.log("We are connected!");

        var manager = nipplejs.create({
            zone: document.getElementById('camera1'),
            color: 'blue'
        });

        manager.on('added', function(evt, nipple){
            nipple.on('move', function (evt, data) {
                var delta = data.position.x - evt.target.position.x;
                console.log("x is : " + delta );
                app.arm.moveArmByDelta(delta, 0 , 'camera1');
            });
        });

    });





}