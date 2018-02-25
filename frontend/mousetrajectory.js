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

    var paper1 = Snap("#svgCam1");          // SVG overlay for Top canvas
    var x1,y1, newX1, newY1;

    // Triangle arrowhead
    var arrow1 = paper1.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
    var marker1 = arrow1.marker(0,0, 10,10, 0,5);

    // center of the screen for the origin of the mouse curve trajectory of first camera perspective
    var startLine1X=this.app.dimCam1Width/2, startLine1Y=this.app.dimCam1Height/2;

    // center of the screen for the origin of the mouse curve trajectory of second camera perspective
    var startLine2X=this.app.dimCam2Width/2, startLine2Y=this.app.dimCam2Height/2;

    var endLine1X=this.app.dimCam1Width, endLine1Y=this.app.dimCam1Height;

    var stX=100, stY=100;
    var firstTime=0;
    var Rad1 =Math.sqrt(Math.pow((stX-startLine1X), 2)+Math.pow((stY-startLine1Y), 2));

    var line1 = paper1.line(startLine1X,startLine1Y,stX,stY)
        .attr({strokeWidth:5,stroke:"red",strokeLinecap:"round",markerEnd: marker1});
    var circle1 = paper1.circle(stX,stY,10).attr({fill:"red"});
    var angle=Math.atan2(stY - startLine1Y, stX - startLine1X)* 180 / Math.PI;
    var endT1X= startLine1X+Rad1;//*Math.cos(90+angle * Math.PI / 180);
    var endT1Y=startLine1Y;//+Rad1*Math.sin(90+angle * Math.PI / 180);
    var circle3 = paper1.circle(endT1X,endT1Y,10).attr({fill:"#50ac1c"});
    var line3 = paper1.line(startLine1X,startLine1Y,100,100)
        .attr({strokeWidth:5,stroke:"#50ac1c",strokeLinecap:"round",markerEnd: marker1});
    var paper1 = Snap("#svgCam1");          // SVG overlay for Top canvas
    var x1,y1, newX1, newY1;

    // Triangle arrowhead
    var arrow1 = paper1.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
    var marker1 = arrow1.marker(0,0, 10,10, 0,5);

    // center of the screen for the origin of the mouse curve trajectory of first camera perspective
    var startLine1X=this.app.dimCam1Width/2, startLine1Y=this.app.dimCam1Height/2;

    // center of the screen for the origin of the mouse curve trajectory of second camera perspective
    var startLine2X=this.app.dimCam2Width/2, startLine2Y=this.app.dimCam2Height/2;

    var endLine1X=this.app.dimCam1Width, endLine1Y=this.app.dimCam1Height;

    var stX=100, stY=100;
    var firstTime=0;
    var Rad1 =Math.sqrt(Math.pow((stX-startLine1X), 2)+Math.pow((stY-startLine1Y), 2));

    //   stX=startLine1X-Rad1, stY=startLine1Y;
    var line1 = paper1.line(startLine1X,startLine1Y,stX,stY).attr({strokeWidth:5,stroke:"red",strokeLinecap:"round",markerEnd: marker1});
    var circle1 = paper1.circle(stX,stY,10).attr({fill:"red"});
    var angle=Math.atan2(stY - startLine1Y, stX - startLine1X)* 180 / Math.PI;
    var endT1X= startLine1X;//+Rad1*Math.cos(90+angle * Math.PI / 180);
    var endT1Y=startLine1Y-Rad1;//+*Math.sin(90+angle * Math.PI / 180);

    var ctl3X=2*startLine1X-0.5*endT1X-0.5*stX, ctl3Y=2*startLine1Y-0.5*endT1Y-0.5*stY;
    stX=startLine1X+Rad1*Math.cos(0* Math.PI/180), stY=startLine1Y+Rad1*Math.sin(0* Math.PI / 180);

    var ctl3X2=2*startLine1X-0.5*endT1X-0.5*stX, ctl3Y2=2*startLine1Y-0.5*endT1Y-0.5*stY;

    stX=startLine1X+Rad1*Math.cos(45* Math.PI / 180);
    stY=startLine1Y+Rad1*Math.sin(45* Math.PI / 180);
    ctl3X2=2*startLine1X-0.5*endT1X-0.5*stX, ctl3Y2=2*startLine1Y-0.5*endT1Y-0.5*stY;
    var svgParabola8="M"+stX+","+stY +" Q"+ctl3X2+","+ctl3Y2+" "+endT1X+","+endT1Y;

    var loopLength = Snap.path.getTotalLength(svgParabola8);

    var marker8 = arrow1.marker(0,0, 10,10, 0,5)
        .attr({markerUnits:'strokeWidth', markerWidth:20,markerHeight:35,orient:"45" });
    var parabola8=paper1.path(Snap.path.getSubpath(svgParabola8, 0, loopLength))
        .attr({ fill: "none", stroke: "#e128ff", opacity: "1" , 'strokeWidth': 3,markerStart: marker8});
var circleParabola8= paper1.circle(stX,stY,10).attr({id:'circleParabola8',fill:"#e128ff"});

    stX=100, stY=100;
    var angle=Math.atan2(stY - startLine1Y, stX - startLine1X)* 180 / Math.PI;
    line3.animate({x2:endT1X,y2:endT1Y}, 2000);
    stX=startLine1X+Rad1*Math.cos(angle* Math.PI / 180);
    stY=startLine1Y+Rad1*Math.sin(angle* Math.PI / 180);

    var mouseRad1 =1;//
    var mouseCircle1 = paper1.circle(startLine1X,startLine1Y,mouseRad1)
        .attr({fill:"green", "fill-opacity": 0.21,stroke: '#123456', 'strokeWidth': 10});

    var gAll = paper1.group();
    var svgString1 ="M"+stX+","+stY +" Q"+ctl3X+","+ctl3Y+" "+endT1X+","+endT1Y;
    svgString1 ="M"+stX+","+stY +" Q"+ctl3X+","+ctl3Y+" "+endT1X+","+endT1Y;
 var midX=(startLine1X-endT1X)/2;

    var w=(startLine1Y-stY)/Math.pow((startLine1X-stX), 2);   // a
    var midY=w*Math.pow((midX-startLine1X), 2)-startLine1Y;
    var ctl4X=2*midX+0.5*endT1X+0.5*startLine1X, ctl4Y=2*midY+0.5*endT1Y+0.5*startLine1Y;

    var loopLength = Snap.path.getTotalLength(svgString1);

    var c = paper1.path(svgString1).attr({ fill: "none", stroke: "blue", opacity: "1" , 'strokeWidth': 3});
 //svgString1 ="M"+startLine1X+","+startLine1Y +" Q"+ctl4X+","+ctl4Y+" "+stX+","+stY;
    //   svgString1 ="M"+startLine1X+","+startLine1Y +" A"+angleT1X+","+angleT1Y+" 0 0,1 "+stX+","+stY;
    //A rx ry x-axis-rotation large-arc-flag sweep-flag x y
    var c2 =paper1.path({
        path: Snap.path.getSubpath(svgString1, 0, loopLength/2),
        stroke: "#e0009f",
        fillOpacity: 0,
        strokeWidth:5,
        strokeLinecap: "round"
    });

    gAll.append(c);
    var ctl1X=50, ctl1Y=80;
    var w=0;
    function dragStart1() {
        this.attr({ fill: 'blue' });
        x1 = line1.attr("x2");
        y1 = line1.attr("y2");
        startLine1X=line1.attr("x1");
        startLine1Y=line1.attr("y1");
    }

    function dragMove1(dx, dy) {
        newX1=+x1+dx;
        newY1=+y1+dy;
        var m1=(newY1-startLine1Y)/(newX1-startLine1X);
        ctl1X=(x1+newX1)/2;
        ctl1Y=(newX1-x1)/2*(2*x1);
        Rad1 =Math.sqrt(Math.pow((newX1-startLine1X), 2)+Math.pow((newY1-startLine1Y), 2));
        angle=Math.atan2(newY1 - startLine1Y, newX1 - startLine1X)* 180 / Math.PI;
        endT1X=parseFloat(startLine1X+Rad1*Math.cos(90+angle * Math.PI / 180)).toFixed(2);
        endT1Y=parseFloat(startLine1Y+Rad1*Math.sin(90+angle * Math.PI / 180)).toFixed(2);
        console.clear();
        console.log(endT1X);
        console.log(endT1Y);
        //"endT1X="
        //      //   mouseCircle1.animate({cx:newX1,cy:newY1}, 7000);   svgString1 ="M"+startLine1X+","+startLine1Y + "Q"+ctl1X+",+ctl1Y+ "+newX1+ ","+newY1;

        if(firstTime==0){
            mouseRad1 =Math.sqrt(Math.pow((newX1-startLine1X), 2)+Math.pow((newY1-startLine1Y), 2));
        }else{
            mouseRad1 =Math.sqrt(Math.pow((dx), 2)+Math.pow((dy), 2));
        }

        var angle=Math.atan2(newY1 - mouseCircle1.attr('cy'), newX1 - mouseCircle1.attr('cx'))* 180 / Math.PI;

        endLine1X= startLine1X+mouseRad1*Math.cos(angle * Math.PI / 180)* 180 / Math.PI; // 1;
        endLine1Y=startLine1Y;//+Math.sin(angle *Math.PI / 180);
        //    alert (angle);        mouseCircle1.attr({r:mouseRad1});
        var dia=2*mouseRad1;//newX1startLin2*e1Y+2*
        // + mouseCircle1.attr('cy')+Math.cos(angle ) alert('The radius mouseCircle1.attr('cx')+is '+r);99 Math.acos(0.5);mouseRad1 *
        //   svgString1  ="M"+0+","+0 +" Q"+startLine1X+","+startLine1Y*2+" "+endLine1X+","+endLine1Y;
        //     c.attr({d:svgString1});
        circle1.attr({cx:newX1,cy:newY1});
        line1.attr({x2:newX1,y2:newY1});
        circle3.attr({cx:endT1X,cy:endT1Y});
        line3.attr({x2:endT1X,y2:0});
    }

    function dragEnd1() {
        /* */       mouseRad1 =1;
        //   mouseCircle1.attr({r:mouseRad1});
        line1.animate({x1:newX1,y1:newY1}, 7000);
        //  mouseCircle1.animate({cx:newX1,cy:newY1}, 7000);
        svgString1 ="M"+newX1+ ","+newY1+" Q"+248+","+100 +newX1+ ","+newY1;
        //   c.animate({d:svgString1}, 7000);
        firstTime=1;
    }
    function dragStartPara() {
       this.attr({ fill: 'blue' });
   //     "M"+stX+","+stY +" Q"+ctl3X2+","+ctl3Y2+" "+endT1X+","+endT1Y;
        x1 = circleParabola8.attr("cx");
        y1 =circleParabola8.attr("cy");
        endT1X= startLine1X+Rad1;//*Math.cos(90+angle * Math.PI / 180);
        endT1Y=startLine1Y;//+Rad1*Math.sin(90+angle * Math.PI / 180);

        ctl3X2=2*startLine1X-0.5*endT1X-0.5*x1, ctl3Y2=2*startLine1Y-0.5*endT1Y-0.5*y1;
        svgParabola8="M"+x1+","+y1 +" Q"+ctl3X2+","+ctl3Y2+" "+endT1X+","+endT1Y;

        parabola8.attr({d:Snap.path.getSubpath(svgParabola8, 0, loopLength/2)});
   // circleParabola8= paper1.circle(x1,y1,10).attr({id:'circleParabola8',fill:"#e128ff"});

    }

    function dragMovePara(dx, dy) {
        newX1=+x1+dx;
        newY1=+y1+dy;
        angle=Math.atan2(newY1 - startLine1Y, newX1 - startLine1X)* 180 / Math.PI;
        Rad1 =Math.sqrt(Math.pow((newX1-startLine1X), 2)+Math.pow((newY1-startLine1Y), 2));
       // stX=startLine1X+Rad1*Math.cos(315* Math.PI / 180);
    //    stY=startLine1Y+Rad1*Math.sin(315* Math.PI / 180);

        endT1X= startLine1X+Rad1;//*Math.cos(angle * Math.PI / 180);
        endT1Y=startLine1Y;//+Rad1*Math.sin(angle * Math.PI / 180);

        ctl3X2=2*startLine1X-0.5*endT1X-0.5*newX1, ctl3Y2=2*startLine1Y-0.5*endT1Y-0.5*newY1;

            //="M"+newX1+ "," + newY1+ " L"+startLine1X+","+startLine1Y+" L" +endT1X+ "," + endT1Y+" Z";
        svgParabola8 ="M"+newX1+ ","+newY1+" Q"+ctl3X2+","+ctl3Y2 + " "+endT1X+","+endT1Y;
//  parabola8.animate({d:Snap.path.getSubpath(svgParabola8, 0, loopLength/2)}, 7000);
       loopLength = Snap.path.getTotalLength(svgParabola8);

        marker8.attr({orient:angle});
          parabola8.attr({d:Snap.path.getSubpath(svgParabola8, 0, loopLength/2)});
                circleParabola8.attr({cx:newX1,cy:newY1});
                //      parabola8.({d:svgParabola8});
/*    /*             myAnim=parabola8.animate({d:Snap.path.getSubpath(svgParabola8, 0, loopLength/2)}, 9000,
              function()
              {        marker8.animate({orient:angle}, 7000);
                  svgParabola8 ="M"+newX1+ ","+newY1+" Q"+newX1+","+newY1 + " "+newX1+","+newY1;
                parabola8.animate({d:Snap.path.getSubpath(svgParabola8, 0, loopLength/2)}, 9000);
              });*/
}

function dragEndPara() {

    svgParabola8 ="M"+newX1+ ","+newY1+" Q"+newX1+","+newY1 + " "+newX1+","+newY1;
    parabola8.animate({d:Snap.path.getSubpath(svgParabola8, 0, loopLength/2)}, 9000);
  startLine1X=newX1;

  startLine1Y=newY1;
//  line2.animate({x1:newX2,y1:newY2}, 7000);
 // Snap.path.getSubpath(loop, 0, step),
//    marker8.attr({});
  //alert("startLine1X="+startLine1X+" startLine1Y="+startLine1Y);
 // alert("ctl3X2="+ctl3X2+" ctl3Y2="+ctl3Y2);
//  alert("newX1="+newX1+" newY1="+newY1);
//
//       endT1X= startLine1X+Rad1;//*Math.cos(90+angle * Math.PI / 180);
//     endT1Y=startLine1Y;//+Rad1*Math.sin(90+angle * Math.PI / 180);

 //
 //
 // parabola8.animate({d:svgParabola8}, 7000);
 // line1.attr({x2:newX1,y2:newY1});
//   circle3.attr({cx:endT1X,cy:endT1Y});
//     line3.attr({x2:endT1X,y2:0});
}

circleParabola8.drag(dragMovePara, dragStartPara, dragEndPara);

var paper2 = Snap("#svgCam2");
var x2,y2, newX2, newY2;


    // Triangle arrowhead
    var arrow2 = paper2.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
    var marker2 = arrow2.marker(0,0, 10,10, 0,5);

var line2 = paper2.line(startLine2X,startLine2Y,100,100)
  .attr({strokeWidth:5,stroke:"green",strokeLinecap:"round",markerEnd: marker2});

var startLine2X=this.app.dimCam2Width/2, startLine2Y=this.app.dimCam2Height/2;

var circle2 = paper2.circle(100,100,10).attr({fill:"green"});
var oval = paper2.ellipse(50, 50, 40, 20).attr({fill:"#027176"});
   // M 0 50 L 10 50 A 40 20, 0, 0 0, 90 50 L 100 50
var ovalStart1 = paper2.circle(300,100,10).attr({fill:"#ab41f6"});
var ovalEnd1 = paper2.circle(100,300,10).attr({fill:"#f27176"});
var centerOval = paper2.ellipse(startLine2X, startLine2Y, 40, 20).attr({fill:"#ffa229",
    fillOpacity: 0.9
    ,opacity:0.7});

    var firstOrbit = paper2.path("M 100, 100 m -75, 0 a 75,75 0 1,0 150,0 a 75,75 0 1,0 -150,0")
        .attr({stroke: "#000", strokeWidth:"5", fill: "transparent"});

    var len = firstOrbit.getTotalLength();
    var circleA = paper2.circle(0,0,10);

    var group = paper2.g(firstOrbit,circleA);
    group = group.transform('r75,400,400');

    var startingPointA = Snap.path.getPointAtLength(firstOrbit, 0);
    var pt = paper2.node.createSVGPoint();
    Snap.animate(0, len, function(value){
        var movePoint = firstOrbit.getPointAtLength(value);
        circleA.attr({ cx: movePoint.x, cy: movePoint.y });
    }, 2000, mina.linear);

    var rect = paper2.rect(0, 0, 10, 10);
    Snap.animate(20, 80, function (val) {
        rect.attr({
            x: val
        });
    }, 1000);

    var circ;

//animation
    var inAttr =  {r: 10, fill:'#5e332a'};
    var outAttr = {r: 30, fill:'#c11a0f'};
    var outAttr2 = {r: 45, fill:'#5dff5e'};
    function animIn(){
        this.animate(inAttr, 1000, mina.linear, animOut);
    }
    function animOut(){
        this.animate(outAttr,1000, mina.linear, animOut2);
    };

    function animOut2(){
        this.animate(outAttr2,1000, mina.linear, animIn);
    };

// main
    circ = paper2.circle(50,60,10)
        .attr(inAttr)
        .animate(outAttr, 1000, mina.linear, animIn);

    startLine2X=startLine2X-40;
    var svg1="M"+startLine2X+","+startLine2Y+"a40,40 0 1,0 80,0a40,40 0 1,0 -80,0";
    var svg2="M"+startLine2X+","+startLine2Y+"a40,50 0 1,0 80,0a40,50 0 1,0 -80,0";
    var svg3="M"+startLine2X+","+startLine2Y+"a40,68 0 1,0 80,0a40,68 0 1,0 -80,0";
    var inAttr =  {d: svg1, stroke:'#5e332a'};
    var outAttr = {d: svg2, stroke:'#c11a0f'};
    var outAttr2 = {d: svg3, stroke:'#5dff5e'};
    function animIn(){
        this.animate(inAttr, 1000, mina.linear, animOut);
    }
    function animOut(){
        this.animate(outAttr,1000, mina.linear, animOut2);
    };

    function animOut2(){
        this.animate(outAttr2,1000, mina.linear, animIn);
    };


   // startLine2Y=startLine2Y-40;
    var svgParabola10="M"+startLine2X+","+startLine2Y+"a40,40 0 1,0 80,0a40,40 0 1,0 -80,0";
    var loopLength = Snap.path.getTotalLength(svgParabola10);
var marker10 = arrow2.marker(0,0, 10,10, 0,5)
        .attr({markerUnits:'strokeWidth', markerWidth:20,markerHeight:35,orient:"45" });
    var parabola10=paper2.path(Snap.path.getSubpath(svgParabola10, 0, loopLength))
        .attr({ fill: "none", stroke: "#e128ff", opacity: "1" , 'strokeWidth': 3})
        .animate(outAttr, 1000, mina.linear, animIn);


    //var circleParabola8= paper1.circle(stX,stY,10).attr({id:'circleParabola8',fill:"#e128ff"});,markerStart: marker10

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