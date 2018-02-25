/**
 * Created by timadamson on 9/6/17.
 */

var downX;
var downY;


function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;
   // app.ros.on('connection', function () {
        console.log("We are connected!");

       // -this.app.dimCam1Widthapp.initRightClickGripper(); // This adds the right click gripper listener

    var panel1= document.getElementById("canvas1");
    panel1.style.top=-this.app.dimCam1Height+ 'px';
    panel1.style.left=0+ 'px';

    var panel2= document.getElementById("canvas2");
    panel2.style.top=-this.app.dimCam2Height+ 'px';
    panel2.style.left=0+ 'px';

    var panelCam1= document.getElementById("svgCam1");
    panelCam1.style.width=this.app.dimCam1Width+ 'px';
    panelCam1.style.height=this.app.dimCam1Height+ 'px';

    var panelCam2= document.getElementById("svgCam2");
    panelCam2.style.width=this.app.dimCam2Width+ 'px';
    panelCam2.style.height=this.app.dimCam2Height+ 'px';

    var paper1 = Snap("#svgCam1");
    var x1,y1, newX1, newY1;

    var arrow1 = paper1.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
    var marker1 = arrow1.marker(0,0, 10,10, 0,5);

    var line1 = paper1.line(0,0,100,100)
        .attr({strokeWidth:5,stroke:"red",strokeLinecap:"round",markerEnd: marker1});

    var circle1 = paper1.circle(100,100,10).attr({fill:"red"});

    function dragStart1() {
        x1 = line1.attr("x2");
        y1 = line1.attr("y2");
    }

    function dragMove1(dx, dy) {
        newX1=+x1+dx;
        newY1=+y1+dy;
        circle1.attr({cx:newX1,cy:newY1});
        line1.attr({x2:newX1,y2:newY1});
    }

    function dragEnd1() {
        line1.animate({x1:newX1,y1:newY1}, 7000);
    }

    circle1.drag(dragMove1, dragStart1, dragEnd1);
    var paper2 = Snap("#svgCam2");
    var x2,y2, newX2, newY2;
    var arrow2 = paper2.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
    var marker2 = arrow2.marker(0,0, 10,10, 0,5);
    var line2 = paper2.line(0,0,100,100)
        .attr({strokeWidth:5,stroke:"green",strokeLinecap:"round",markerEnd: marker1});

    var circle2 = paper2.circle(100,100,10).attr({fill:"green"});

    function dragStart2() {
        x2 = line2.attr("x2");
        y2 = line2.attr("y2");
    }

    function dragMove2(dx, dy) {
        newX2=+x2+dx;
        newY2=+y2+dy;
        circle2.attr({cx:newX2,cy:newY2});
        line2.attr({x2:newX2,y2:newY2});
    }

    function dragEnd2() {
        line2.animate({x1:newX2,y1:newY2}, 7000);
    }

    circle2.drag(dragMove2, dragStart2, dragEnd2);
    var resizeOverlay = _.debounce(function() {

        panel1.style.top=-this.app.getCam1H()+ 'px';
        panel1.style.left=0+ 'px';

        panelCam1.style.width=this.app.getCam1W()+ 'px';
        panelCam1.style.height=this.app.getCam1H()+ 'px';
        panel2.style.top=-this.app.getCam2H()+ 'px';
        panel2.style.left=0+ 'px';

        panelCam2.style.width=this.app.getCam2W()+ 'px';
        panelCam2.style.height=this.app.getCam2H()+ 'px';

    }, 400);

    window.addEventListener('resize', resizeOverlay);

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
                        console.log("Mouse down happend at x1:" + downX + " , y1:" + downY);
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

   // });


}