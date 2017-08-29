/**
 * Created by timadamson on 8/25/17.
 */
/**
 * Created by timadamson on 8/24/17.
 */

// Public variables
var downX;
var downY;
var cmdVelTopic;
var twist;

/*
 Function.prototype.throttle = function (milliseconds, context) {
 var baseFunction = this,
 lastEventTimestamp = null,
 limit = milliseconds;

 return function () {
 var self = context || this,
 args = arguments,
 now = Date.now();

 if (!lastEventTimestamp || now - lastEventTimestamp >= limit) {
 lastEventTimestamp = now;
 baseFunction.apply(self, args);
 }
 };
 };
 */

var latestPositionX;
var latestPositionY;
var latestTargetX;
var latestTargetY;
var lastInterval;

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
                latestPositionX = data.position.x;
                latestPositionY = data.position.y;
                latestTargetX = evt.target.position.x;
                latestTargetY = evt.target.position.y;
                var deltaX = latestPositionX - latestTargetX;
                var deltaY = latestPositionY - latestTargetY;


                console.log("x is : " + deltaX + "\n" + "and y is :" + deltaY );
                app.arm.moveArmByDelta(deltaX, deltaY , 'camera1');


                if(lastInterval) {
                    clearInterval(lastInterval);
                }

                lastInterval = setInterval(function () {
                    var deltaX = latestPositionX - latestTargetX;
                    var deltaY = latestPositionY - latestTargetY;


                    console.log("x is : " + deltaX + "\n" + "and y is :" + deltaY );
                    app.arm.moveArmByDelta(deltaX/2, deltaY/2 , 'camera1');

                }, 100);



            });
        }).on('removed', function (evt, nipple) {
            if(lastInterval) {
                clearInterval(lastInterval);
            }
        });

    });





}