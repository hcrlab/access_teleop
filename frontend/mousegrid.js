/**
 * Created by Kavita on 1/6/18.
 */
$(document).ready(function() {

    $("#draggableSpeech").draggable().resizable();
});
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
    var oneBTN1 =document.getElementById("oneBTN1");
    var twoBTN1 =document.getElementById("twoBTN1");
    var threeBTN1 =document.getElementById("threeBTN1");
    var fourBTN1 =document.getElementById("fourBTN1");
    var fiveBTN1 =document.getElementById("fiveBTN1");
    var sixBTN1 =document.getElementById("sixBTN1");
    var sevBTN1 =document.getElementById("sevBTN1");
    var eigBTN1 =document.getElementById("eigBTN1");
    var nineBTN1 =document.getElementById("nineBTN1");
    var oneBTN2 =document.getElementById("oneBTN2");
    var twoBTN2 =document.getElementById("twoBTN2");
    var threeBTN2 =document.getElementById("threeBTN2");
    var fourBTN2 =document.getElementById("fourBTN2");
    var fiveBTN2 =document.getElementById("fiveBTN2");
    var sixBTN2 =document.getElementById("sixBTN2");
    var sevBTN2 =document.getElementById("sevBTN2");
    var eigBTN2 =document.getElementById("eigBTN2");
    var nineBTN2 =document.getElementById("nineBTN2");

    oneBTN1.addEventListener("click", numSelect);
    twoBTN1.addEventListener("click", numSelect);
    threeBTN1.addEventListener("click", numSelect);
    fourBTN1.addEventListener("click", numSelect);
    fiveBTN1.addEventListener("click", numSelect);
    sixBTN1.addEventListener("click", numSelect);
    sevBTN1.addEventListener("click", numSelect);
    eigBTN1.addEventListener("click", numSelect);
    nineBTN1.addEventListener("click", numSelect);

    oneBTN2.addEventListener("click", numSelect2);
    twoBTN2.addEventListener("click", numSelect2);
    threeBTN2.addEventListener("click", numSelect2);
    fourBTN2.addEventListener("click", numSelect2);
    fiveBTN2.addEventListener("click", numSelect2);
    sixBTN2.addEventListener("click", numSelect2);
    sevBTN2.addEventListener("click", numSelect2);
    eigBTN2.addEventListener("click", numSelect2);
    nineBTN2.addEventListener("click", numSelect2);
    var numCountLevelCam1= 0;
    var numCountLevelCam2= 0;
    var gridLevelStateCam1= 0;
    var gridLevelStateCam2= 0;
    var gridLevelPrevCam1= 0;
    var gridLevelPrevCam2= 0;
    var firstTimeCam1=0;
    var firstTimeCam2=0;
    var gridLevelStateCam1= 0;
    var gridLevelStateCam2= 0;
    var IDLevelStateCam1=0;
    var IDLevelState2Cam1=0;
    var IDLevelState3Cam1=0;
    var IDLevelStateCam2=0;
    var IDLevelState2Cam2=0;
    var IDLevelState3Cam2=0;

    function numSelect() {
        numCountLevelCam1++;

        GlobalVal.incrCam1Lev();
        var id = this.id;
        var value= this.text;
        var grabGroup0, grabGroup1,grabGroup2, grabCircle, grabLine;
        if(id=="oneBTN1" || id=="twoBTN1" || id=="threeBTN1" || id=="fourBTN1" || id=="fiveBTN1"
            || id=="sixBTN1" || id=="sevBTN1" || id=="eigBTN1" || id=="nineBTN1"){

            grabGroup0=gAll.selectAll('*');
            grabGroup1=gLevel1.selectAll('*');
            grabGroup2=gLevel2.selectAll('*');
            grabCircle=circle1;
            grabLine=line1;

        }

        var num =value;

        switch (numCountLevelCam1) {
            case 1:
                IDLevelStateCam1=num;

                grabGroup0.forEach( function( el ) {
                    if (el.data("index")!= num){
                        el.attr({ visibility: "hidden"});
                    }

                    grabGroup1.forEach( function( el2 ) {
                        var selEl =el2.data("parent");
                        if (selEl==num ) {
                            el2.attr({visibility: ""});

                        }else {
                            el2.attr({ visibility: "hidden"});
                        }
                        el.attr({ visibility: "hidden"});
                    });
                });

                firstTimeCam1=1;
                gridLevelStateCam1=1;
                gridLevelPrevCam1 = 0;

                break;
            case 2:

                IDLevelState2Cam1 = num;
                var parent  =IDLevelStateCam1;

                grabGroup1.forEach( function( el2 ) {
                    if (el2.data("index") == num && el2.data("parent") == parent  ) {

                        if(el2.type=== 'rect') {
                            el2.attr({visibility: ""});
                        } else {
                            el2.attr({visibility: "hidden"});
                        }

                        grabGroup2.forEach( function( el3 ) {
                            var selEl3 =el3.data("parent");
                            if (el3.data("parent") == num &&el3.data("grandparent") == parent ) {
                                el2.attr({visibility: "hidden"});
                                el3.attr({visibility: ""});
                            }
                        });
                    }else{
                        el2.attr({visibility: "hidden"});
                    }
                });

                gridLevelPrevCam1 =1;
                gridLevelStateCam1=2;
                break;
            case 3:
                var parent  =IDLevelState2Cam1
                var grandparent  =IDLevelStateCam1;
                IDLevelState3Cam1= num;

                grabGroup2.forEach( function( el3 ) {
                    if (el3.data("index") == num && el3.data("parent") == parent   && el3.data("grandparent") == grandparent  ) {
                        var dimens = el3.node.getBBox();
                        var middleX = dimens.x + (dimens.width / 2),
                            middleY = dimens.y + (dimens.height / 2);
                        grabCircle.attr({cx:middleX,cy:middleY});
                        grabLine.attr({x2:middleX,y2:middleY});
                        grabLine.animate({x1:middleX,y1:middleY}, 7000);

                        console.log("The num is " + num + " the parent is: " + parent + " the grandparent is: " + grandparent);
                        canvasNumber = el3.node.id[el3.node.id.length - 1];
                        cameraName = $(el3.node).closest("#canvas" + canvasNumber).siblings("#camera" + canvasNumber)[0].id;
                        self.app.arm.moveArmByAbsolute(middleX, middleY, cameraName)
                        el3.animate({},  7000,
                            function(){
                                el3.attr({visibility: "hidden"});
                                grabGroup0.forEach( function( el ) {
                                    el.attr({ visibility: ""});
                                });
                            }
                        );
                    }
                    else{
                        el3.attr({visibility: "hidden"});
                    }
                });

                gridLevelPrevCam1 = 0;
                gridLevelStateCam1=0;
                firstTimeCam1=0;
                numCountLevelCam1 = 0;
                break;
        }

    }

    function numSelect2() {
        numCountLevelCam2++;
        GlobalVal.incrCam2Lev();

        var id = this.id;
        var value= this.text;
        var grabGroup0, grabGroup1,grabGroup2, grabCircle, grabLine;
        if(id=="oneBTN2" || id=="twoBTN2" || id=="threeBTN2" || id=="fourBTN2" || id=="fiveBTN2"
            || id=="sixBTN2" || id=="sevBTN2" || id=="eigBTN2" || id=="nineBTN2"){

            grabGroup0=gAllcam2.selectAll('*');
            grabGroup1=gLevel1cam2.selectAll('*');
            grabGroup2=gLevel2cam2.selectAll('*');
            grabCircle=circle2;
            grabLine=line2;
        }

        var num =value;

        switch (numCountLevelCam2) {
            case 1:
                IDLevelStateCam2=num;

                grabGroup0.forEach( function( el ) {
                    if (el.data("index")!= num){
                        el.attr({ visibility: "hidden"});
                    }

                    grabGroup1.forEach( function( el2 ) {
                        var selEl =el2.data("parent");
                        if (selEl==num ) {
                            el2.attr({visibility: ""});
                        }else {
                            el2.attr({ visibility: "hidden"});
                        }
                        el.attr({ visibility: "hidden"});
                    });
                });

                firstTimeCam2=1;
                gridLevelStateCam2=1;
                gridLevelPrevCam2 = 0;

                break;
            case 2:
                IDLevelState2Cam2 = num;
                var parent  =IDLevelStateCam2;

                grabGroup1.forEach( function( el2 ) {
                    if (el2.data("index") == num &&el2.data("parent") == parent  ) {

                        if(el2.type=== 'rect') {
                            el2.attr({visibility: ""});
                        } else {
                            el2.attr({visibility: "hidden"});
                        }

                        grabGroup2.forEach( function( el3 ) {
                            var selEl3 =el3.data("parent");
                            if (el3.data("parent") == num &&el3.data("grandparent") == parent ) {
                                el2.attr({visibility: "hidden"});
                                el3.attr({visibility: ""});
                            }
                        });
                    }else{
                        el2.attr({visibility: "hidden"});
                    }
                });

                gridLevelPrevCam2 =1;
                gridLevelStateCam2=2;
                break;
            case 3:
                var parent  =IDLevelState2Cam2;
                var grandparent  =IDLevelStateCam2;
                IDLevelState3Cam2= num;

                grabGroup2.forEach( function( el3 ) {
                    if (el3.data("index") == num && el3.data("parent") == parent   && el3.data("grandparent") == grandparent  ) {
                        var dimens = el3.node.getBBox();
                        var middleX = dimens.x + (dimens.width / 2),
                            middleY = dimens.y + (dimens.height / 2);
                        grabCircle.attr({cx:middleX,cy:middleY});
                        grabLine.attr({x2:middleX,y2:middleY});
                        grabLine.animate({x1:middleX,y1:middleY}, 7000);

                        console.log("The num is " + num + " the parent is: " + parent + " the grandparent is: " + grandparent);
                        console.log("middleX : " + middleX + " middleY: " + middleY);
                        canvasNumber = el3.node.id[el3.node.id.length - 1];
                        cameraName = $(el3.node).closest("#canvas" + canvasNumber).siblings("#camera" + canvasNumber)[0].id;
                        self.app.arm.moveArmByAbsolute(middleX, middleY, cameraName)

                        el3.animate({},  7000,
                            function(){
                                el3.attr({visibility: "hidden"});
                                grabGroup0.forEach( function( el ) {
                                    el.attr({ visibility: ""});
                                });
                            }
                        );
                    }
                    else{
                        el3.attr({visibility: "hidden"});
                    }
                });

                gridLevelPrevCam2 = 0;
                gridLevelStateCam2=0;
                firstTimeCam2=0;
                numCountLevelCam2 = 0;
                break;
        }
       // alert("numCountLevelCam2="+numCountLevelCam2);
    }

    var elbackBTN1 = document.getElementById("backBTN1");
    elbackBTN1.addEventListener("click", gridBackLevel);
    var elbackBTN2 = document.getElementById("backBTN2");
    elbackBTN2.addEventListener("click", gridBackLevel2);

    var elforwardBTN1 = document.getElementById("forwardBTN1");
    elforwardBTN1.addEventListener("click", gridForwardLevel);
    var elforwardBTN2 = document.getElementById("forwardBTN2");
    elforwardBTN2.addEventListener("click", gridForwardLevel2);

    function gridBackLevel (){
        numCountLevelCam1--;

        GlobalVal.decrCam1Lev();
        var id=this.id;

        var grabGroup0, grabGroup1,grabGroup2;
        if(id=="backBTN1"){
            grabGroup0=gAll.selectAll('*');
            grabGroup1=gLevel1.selectAll('*');
            grabGroup2=gLevel2.selectAll('*');
        }

        if (gridLevelStateCam1==0){
            alert("Top: no more level back");
        }else if (gridLevelStateCam1==1){
            grabGroup0.forEach( function( el ) {
                el.attr({ visibility: ""});
            });

            grabGroup1.forEach( function( el2 ) {
                el2.attr({ visibility: "hidden"});
            });

            gridLevelPrevCam1= 1;
            gridLevelStateCam1=0;

        }else if (gridLevelStateCam1==2){
            grabGroup1.forEach( function( el ) {
                if (el.data("parent")== IDLevelStateCam1){
                    el.attr({ visibility: ""});
                }
            });
            grabGroup2.forEach( function( el2 ) {
                el2.attr({ visibility: "hidden"});
            });
            gridLevelPrevCam1= 2;
            gridLevelStateCam1=1;
        }
    }

    function gridBackLevel2(){
        numCountLevelCam2--;
        GlobalVal.decrCam2Lev();
        var id=this.id;

        var grabGroup0, grabGroup1,grabGroup2;
        if(id=="backBTN2"){
            grabGroup0=gAllcam2.selectAll('*');
            grabGroup1=gLevel1cam2.selectAll('*');
            grabGroup2=gLevel2cam2.selectAll('*');
        }

        if (gridLevelStateCam2==0){
            alert("Side: No more level back");
        }else if (gridLevelStateCam2==1){
            grabGroup0.forEach( function( el ) {
                el.attr({ visibility: ""});
            });

            grabGroup1.forEach( function( el2 ) {
                el2.attr({ visibility: "hidden"});
            });

            gridLevelPrevCam2= 1;
            gridLevelStateCam2=0;

        }else if (gridLevelStateCam2==2){
            grabGroup1.forEach( function( el ) {
                if (el.data("parent")== IDLevelStateCam2){
                    el.attr({ visibility: ""});
                }
            });
            grabGroup2.forEach( function( el2 ) {
                el2.attr({ visibility: "hidden"});
            });
            gridLevelPrevCam2= 2;
            gridLevelStateCam2=1;

        }
    }

    function gridForwardLevel (){
        numCountLevelCam1++;

        GlobalVal.incrCam1Lev();
        var id=this.id;

        var grabGroup0, grabGroup1,grabGroup2;
        if(id=="forwardBTN1") {
            grabGroup0 = gAll.selectAll('*');
            grabGroup1 = gLevel1.selectAll('*');
            grabGroup2 = gLevel2.selectAll('*');
        }

        if (gridLevelPrevCam1==0 && firstTimeCam1==0){
            alert("Top: No more level forward");

        }else if (gridLevelPrevCam1==0 && firstTimeCam1==1){   // there is another level forward

            grabGroup1.forEach( function( el ) {
                el.attr({ visibility: "hidden"});
            });
            grabGroup2.forEach( function( el2 ) {
                if (el2.data("parent") ==IDLevelState2Cam1  &&el2.data("grandparent")== IDLevelStateCam1 ){
                    el2.attr({ visibility: ""});
                }

            });
            gridLevelPrevCam1=1;
            gridLevelStateCam1=2;

        }
        else if (gridLevelPrevCam1==1){
            grabGroup0.forEach( function( el ) {
                el.attr({ visibility: "hidden"});
            });

            grabGroup1.forEach( function( el2 ) {
                if (el2.data("parent")== IDLevelStateCam1){
                    el2.attr({ visibility: ""});
                }
            });

            gridLevelPrevCam1=0;
            gridLevelStateCam1=1;
        }else if (gridLevelPrevCam1==2){
            grabGroup1.forEach( function( el ) {
                el.attr({ visibility: "hidden"});
            });
            grabGroup2.forEach( function( el2 ) {
                if (el2.data("parent") ==IDLevelState2Cam1  &&el2.data("grandparent")== IDLevelStateCam1 ){
                    el2.attr({ visibility: ""});
                }
            });

            gridLevelPrevCam1=1;
            gridLevelStateCam1=2;
        }
        //   alert("forward: Curr ="+gridLevelStateCam1+" Prev ="+gridLevelPrevCam1);
    }

    function gridForwardLevel2(){
        numCountLevelCam2++;
        GlobalVal.incrCam2Lev();
        var id=this.id;

        var grabGroup0, grabGroup1,grabGroup2;
        if(id=="forwardBTN2") {
            grabGroup0 = gAllcam2.selectAll('*');
            grabGroup1 = gLevel1cam2.selectAll('*');
            grabGroup2 = gLevel2cam2.selectAll('*');
        }

        if (gridLevelPrevCam2==0 && firstTimeCam2==0){
            alert("Side: No more level forward");

        }else if (gridLevelPrevCam2==0 && firstTimeCam2==1){   // there is another level forward

            grabGroup1.forEach( function( el ) {
                el.attr({ visibility: "hidden"});
            });
            grabGroup2.forEach( function( el2 ) {
                if (el2.data("parent") ==IDLevelState2Cam2  && el2.data("grandparent")== IDLevelStateCam2){
                    el2.attr({ visibility: ""});
                }
            });
            gridLevelPrevCam2=1;
            gridLevelStateCam2=2;

        } else if (gridLevelPrevCam2==1){
            grabGroup0.forEach( function( el ) {
                el.attr({ visibility: "hidden"});
            });

            grabGroup1.forEach( function( el2 ) {
                if (el2.data("parent")== IDLevelStateCam2){
                    el2.attr({ visibility: ""});
                }
            });

            gridLevelPrevCam2=0;
            gridLevelStateCam2=1;
        }else if (gridLevelPrevCam2==2){
            grabGroup1.forEach( function( el ) {
                el.attr({ visibility: "hidden"});
            });
            grabGroup2.forEach( function( el2 ) {
                if (el2.data("parent") ==IDLevelState2Cam2 && el2.data("grandparent")== IDLevelStateCam2 ){
                    el2.attr({ visibility: ""});
                }
            });

            gridLevelPrevCam2=1;
            gridLevelStateCam2=2;
        }
        //   alert("forward: Curr ="+gridLevelStateCam1+" Prev ="+gridLevelPrevCam1);
    }

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
    var paper2 = Snap("#svgCam2");
    var x2,y2, newX2, newY2;
    var arrow2 = paper2.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
    var marker2 = arrow2.marker(0,0, 10,10, 0,5);
    var line2 = paper2.line(0,0,100,100)
        .attr({strokeWidth:5,stroke:"green",strokeLinecap:"round",markerEnd: marker2});

    var circle2 = paper2.circle(100,100,10).attr({fill:"green"});

    var w =document.getElementById("cameraTop").clientWidth;
    var h =document.getElementById("cameraTop").clientHeight;

    var gAll = paper1.group();
    var gLevel1 = paper1.group();
    var gLevel2 = paper1.group();

    var gAllcam2= paper2.group();
    var gLevel1cam2 = paper2.group();
    var gLevel2cam2 = paper2.group();
    createBlocks();

    function gridSel() {

        numCountLevelCam1++;
        var num =this.data("index");
        IDLevelStateCam1=num;
        var id=this.attr("id");
        var grabGroup0, grabGroup1;
        if(id=="paper1"){
            grabGroup0=gAll.selectAll('*');
            grabGroup1=gLevel1.selectAll('*');

            GlobalVal.incrCam1Lev();
   //         alert("levelCam1 =" +GlobalVal.getCam1Lev);
        }else{
            grabGroup0=gAllcam2.selectAll('*');
            grabGroup1=gLevel1cam2.selectAll('*');

            GlobalVal.incrCam2Lev();
     //       alert("levelCam2 =" +GlobalVal.getCam2Lev);
        }
        grabGroup0.forEach( function( el ) {
                if (el.data("index")!= num){
                    el.attr({ visibility: "hidden"});
                }

                grabGroup1.forEach( function( el2 ) {
                        var selEl =el2.data("parent");
                  if (selEl==num ) {
                            el2.attr({visibility: ""});

                  }else {
                      el2.attr({ visibility: "hidden"});
                  }
                        el.attr({ visibility: "hidden"});
                    });
            });

        firstTimeCam1=1;
        gridLevelStateCam1=1;
        gridLevelPrevCam1 = 0;
    ////    alert("forward: Curr ="+gridLevelStateCam1+" Prev ="+gridLevelPrevCam1);
    }

    function gridSelL1() {
        numCountLevelCam1++;
        var num =this.data("index");
        IDLevelState2Cam1 = num;
        var parent  =this.data("parent");
        var id=this.attr("id");
        var grabGroup1, grabGroup2;
        if(id=="paper1"){
            grabGroup1=gLevel1.selectAll('*');
            grabGroup2=gLevel2.selectAll('*');

            GlobalVal.incrCam1Lev();
     //       alert("levelCam1 =" +GlobalVal.getCam1Lev);
        }else{
            grabGroup1=gLevel1cam2.selectAll('*');
            grabGroup2=gLevel2cam2.selectAll('*');

            GlobalVal.incrCam2Lev();
            //alert("levelCam2 =" +GlobalVal.getCam2Lev);
        }

        grabGroup1.forEach( function( el2 ) {
                        if (el2.data("index") == num &&el2.data("parent") == parent  ) {

                            if(el2.type=== 'rect') {
                                el2.attr({visibility: ""});
                            } else {
                                el2.attr({visibility: "hidden"});
                            }

        grabGroup2.forEach( function( el3 ) {
                                    var selEl3 =el3.data("parent");
                             if (el3.data("parent") == num &&el3.data("grandparent") == parent ) {
                                 el2.attr({visibility: "hidden"});
                                 el3.attr({visibility: ""});
                                    }
                                });
                        }else{

                            el2.attr({visibility: "hidden"});
                        }
                    });

        gridLevelPrevCam1 =1;
        gridLevelStateCam1=2;

    }

    function gridSelL2() {
        numCountLevelCam1++;
        var num =this.data("index");
        var parent  =this.data("parent");
        var grandparent  =this.data("grandparent");
        IDLevelState3Cam1= num;

        var id=this.attr("id");
        var grabGroup0, grabGroup2, grabCircle, grabLine;
        if(id=="paper1"){
            grabGroup0=gAll.selectAll('*');
            grabGroup2=gLevel2.selectAll('*');
            grabCircle=circle1;
            grabLine=line1;

            GlobalVal.resetCam1Lev();
      //      alert("Reseted levelCam1 =" +GlobalVal.getCam1Lev);

        }else{
            grabGroup0=gAllcam2.selectAll('*');
            grabGroup2=gLevel2cam2.selectAll('*');
            grabCircle=circle2;
            grabLine=line2;

            GlobalVal.resetCam2Lev();
        //    alert("Reseted levelCam2 =" +GlobalVal.getCam2Lev);

        }

        grabGroup2.forEach( function( el3 ) {
                if (el3.data("index") == num && el3.data("parent") == parent   && el3.data("grandparent") == grandparent  ) {
                    var dimens = el3.node.getBBox();
                    var middleX = dimens.x + (dimens.width / 2),
                        middleY = dimens.y + (dimens.height / 2);
                    grabCircle.attr({cx:middleX,cy:middleY});
                    grabLine.attr({x2:middleX,y2:middleY});
                    grabLine.animate({x1:middleX,y1:middleY}, 7000);

                    console.log("The num is " + num + " the parent is: " + parent + " the grandparent is: " + grandparent);
                    canvasNumber = el3.node.id[el3.node.id.length - 1];
                    cameraName = $(el3.node).closest("#canvas" + canvasNumber).siblings("#camera" + canvasNumber)[0].id;
                    self.app.arm.moveArmByAbsolute(middleX, middleY, cameraName)

                    el3.animate({},  7000,
                        function(){
                            el3.attr({visibility: "hidden"});
                            grabGroup0.forEach( function( el ) {
                                        el.attr({ visibility: ""});
                                });
                        }
                    );
                }
                else{
                    el3.attr({visibility: "hidden"});
                }
            });

        gridLevelPrevCam1 = 0;
        gridLevelStateCam1=0;
        firstTimeCam1=0;

    }

    var numGrid=0;
    var numGrid2=0;
    function createBlocks() {
        for (var i = 0; i < 3 ; i++) {
            for (var j = 0; j < 3; j++) {
                if (j == 0) {
                    numGrid = i + 1;
                } else if (j == 1) {
                    numGrid = 3 + i + 1;
                } else if (j == 2) {
                    numGrid = 6 + i + 1;

                }
                var block1 = paper1.rect(i * w / 3, j * h / 3, w / 3, h / 3)
                    .attr({fill: "#fc0", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.1})
                    .data("index", numGrid)
                    .click(gridSel);
                block1.attr({id: 'paper1'});
                var block1cam2 = paper2.rect(i * w / 3, j * h / 3, w / 3, h / 3)
                    .attr({fill: "#fc0", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.1})
                    .data("index", numGrid)
                    .click(gridSel);

                block1cam2.attr({id: 'paper2'});

                var ninthBlock = paper2.rect(w-w / 3, 2 * h / 3, w / 3, h / 3)
                    .attr({fill: "#fc0", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.01})
                    .data("index", numGrid)
                    .click(gridSel);

                ninthBlock.attr({id: 'paper2'});

                var block1text = paper1.text(w / 10 + i * w / 3, h / 4 + j * h / 3, numGrid);
                block1text.attr({
                    'font-size': 100
                });
                block1text.attr({id: 'paper1'});

                line1.paper.append(line1);
                circle1.paper.append(circle1);

                line2.paper.append(line2);
                circle2.paper.append(circle2);

                var block1textcam2 = paper2.text(w / 10 + i * w / 3, h / 4 + j * h / 3, numGrid);
                block1textcam2.attr({
                    'font-size': 100
                });

                block1textcam2.attr({id: 'paper2'});

                block1text.data("index", numGrid);
                block1text.click(gridSel);

                block1textcam2.data("index", numGrid);
                block1textcam2.click(gridSel);

                block1.data("parent", 0);
                block1.data("grandparent", 0);
                ninthBlock.data("grandparent", 0);
                ninthBlock.data("parent", 0);
                block1text.data("parent", 0);
                block1text.data("grandparent", 0);
                block1textcam2.data("parent", 0);
                block1textcam2.data("grandparent", 0);
                gAll.append(block1);
                gAll.append(block1text);

                gAllcam2.append(ninthBlock);
                gAllcam2.append(block1cam2);
                gAllcam2.append(block1textcam2);
            }
        }

        for (var k = 0; k < 9; k++) {
            for (var m = 0; m < 9; m++) {

                if (k == 0 || k == 3 || k == 6) {
                    if (m == 0 || m == 3 || m == 6) {
                        numGrid2 = 1;
                    } else if (m == 1 || m == 4 || m == 7) {
                        numGrid2 = 2;
                    } else if (m == 2 || m == 5 || m == 8) {
                        numGrid2 = 3;
                    }
                } else if (k == 1 || k == 4 || k == 7) {
                    if (m == 0 || m == 3 || m == 6) {
                        numGrid2 = 4;
                    } else if (m == 1 || m == 4 || m == 7) {
                        numGrid2 = 5;
                    } else if (m == 2 || m == 5 || m == 8) {
                        numGrid2 = 6;
                    }
                } else if (k == 2 || k == 5 || k == 8) {
                    if (m == 0 || m == 3 || m == 6) {
                        numGrid2 = 7;
                    } else if (m == 1 || m == 4 || m == 7) {
                        numGrid2 = 8;
                    } else if (m == 2 || m == 5 || m == 8) {
                        numGrid2 = 9;
                    }
                }


                var block2 = paper1.rect(m * w / 9, k * h / 9, w / 9, h / 9)
                    .attr({visibility: "hidden", fill: "#330066", stroke: "#330066", strokeWidth: 2})
                    .data("index", numGrid2);
                block2.click(gridSelL1);

                var text = paper1.text(w / 9 / 2 + m * w / 9, h / 14 + k * h / 9, numGrid2);
                text.attr({
                    visibility: "hidden",
                    'font-size': 30
                });

                block2.attr({id: 'paper1'});
                text.attr({id: 'paper1'});

                var block2cam2 = paper2.rect(m * w / 9, k * h / 9, w / 9, h / 9)
                    .attr({visibility: "hidden", fill: "#330066", stroke: "#330066", strokeWidth: 2})
                    .data("index", numGrid2);
                block2cam2.click(gridSelL1);

                block2cam2.attr({id: 'paper2'});

                var textcam2 = paper2.text(w / 9 / 2 + m * w / 9, h / 14 + k * h / 9, numGrid2);
                textcam2.attr({
                    visibility: "hidden",
                    'font-size': 30
                });
                textcam2.attr({id: 'paper2'});

                block2.data("grandparent", 0);
                text.data("grandparent", 0);
                text.data("index", numGrid2);
                text.click(gridSelL1);

                block2cam2.data("grandparent", 0);
                textcam2.data("grandparent", 0);
                textcam2.data("index", numGrid2);
                textcam2.click(gridSelL1);
                textcam2.append(block1cam2);
                if (m < 3 && k < 3) {
                    block2cam2.attr({fill: "#ffb912"});
                    block2cam2.data("parent", 1);
                    textcam2.data("parent", 1);
                    block2.attr({fill: "#ffb912"});
                    block2.data("parent", 1);
                    text.data("parent", 1);
                } else if (m >= 3 && m < 6 && k < 3) {
                    block2.attr({fill: "#d9ffb2"});
                    block2.data("parent", 2);
                    text.data("parent", 2);
                    block2cam2.attr({fill: "#d9ffb2"});
                    block2cam2.data("parent", 2);
                    textcam2.data("parent", 2);
                } else if (m >= 6 && m < 9 && k < 3) {
                    block2.attr({fill: "#ffd9b2"});
                    block2.data("parent", 3);
                    text.data("parent", 3);
                    block2cam2.attr({fill: "#ffd9b2"});
                    block2cam2.data("parent", 3);
                    textcam2.data("parent", 3);
                } else if (m < 3 && k >= 3 && k < 6) {
                    block2.attr({fill: "#a47fff"});
                    block2.data("parent", 4);
                    text.data("parent", 4);
                    block2cam2.attr({fill: "#a47fff"});
                    block2cam2.data("parent", 4);
                    textcam2.data("parent", 4);
                } else if (m >= 3 && m < 6 && k >= 3 && k < 6) {
                    block2.attr({fill: "#ffb2ff"});
                    block2.data("parent", 5);
                    text.data("parent", 5);
                    block2cam2.attr({fill: "#ffb2ff"});
                    block2cam2.data("parent", 5);
                    textcam2.data("parent", 5);
                } else if (m >= 6 && m < 9 && k >= 3 && k < 6) {
                    block2.attr({fill: "#b2ffb2"});
                    block2.data("parent", 6);
                    text.data("parent", 6);
                    block2cam2.attr({fill: "#b2ffb2"});
                    block2cam2.data("parent", 6);
                    textcam2.data("parent", 6);
                } else if (m < 3 && k >= 6 && k < 9) {
                    block2.attr({fill: "#ffffb2"});
                    block2.data("parent", 7);
                    text.data("parent", 7);
                    block2cam2.attr({fill: "#ffffb2"});
                    block2cam2.data("parent", 7);
                    textcam2.data("parent", 7);
                } else if (m >= 3 && m < 6 && k >= 6 && k < 9) {
                    block2.attr({fill: "#b2ffff"});
                    block2.data("parent", 8);
                    ;
                    text.data("parent", 8);
                    block2cam2.attr({fill: "#b2ffff"});
                    block2cam2.data("parent", 8);
                    textcam2.data("parent", 8);
                } else if (m >= 6 && m < 9 && k >= 6 && k < 9) {
                    block2.attr({fill: "#FF6666"});
                    block2.data("parent", 9);
                    text.data("parent", 9);
                    block2cam2.attr({fill: "#FF6666"});
                    block2cam2.data("parent", 9);
                    textcam2.data("parent", 9);
                }

                var dimens = block2.node.getBBox();
                var x = dimens.x;
                var y = dimens.y;
                var dimW = dimens.width / 3;
                var dimH = dimens.height / 3;
                var dimens2 = block2cam2.node.getBBox();
                var x2 = dimens2.x;
                var y2 = dimens2.y;
                var dimW2 = dimens2.width / 3;
                var dimH2 = dimens2.height / 3;

                var a1 = paper1.rect(x, y, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#C5E72B", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 1)
                    .click(gridSelL2);

                var text1 = paper1.text(dimW / 4 + x, dimH + y, 1);
                text1.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text1.data("index", 1);

                a1.attr({id: 'paper1'});
                text1.attr({id: 'paper1'});

                var a1cam2 = paper2.rect(x2, y2, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#C5E72B", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 1)
                    .click(gridSelL2);

                var text1cam2 = paper2.text(dimW2 / 4 + x2, dimH2 + y2, 1);
                text1cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text1cam2.data("index", 1);

                var a2 = paper1.rect(x + dimW, y, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#0d2be7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 2)
                    .click(gridSelL2);
                var text2 = paper1.text(dimW / 4 + x + dimW, dimH + y, 2);
                text2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text2.data("index", 2);

                var a2cam2 = paper2.rect(x2 + dimW2, y2, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#C5E72B", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 2)
                    .click(gridSelL2);

                var text2cam2 = paper2.text(dimW2 / 4 + x2 + dimW2, dimH2 + y2, 2);
                text2cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text2cam2.data("index", 2);

                var a3 = paper1.rect(x + dimW * 2, y, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#e72bc5", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 3)
                    .click(gridSelL2);

                var text3 = paper1.text(dimW / 4 + x + dimW * 2, dimH + y, 3);
                text3.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text3.data("index", 3);

                var a3cam2 = paper2.rect(x2 + dimW2 * 2, y2, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#e72bc5", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 3)
                    .click(gridSelL2);

                var text3cam2 = paper2.text(dimW2 / 4 + x2 + dimW2 * 2, dimH2 + y2, 3);
                text3cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text3cam2.data("index", 3);

                // second row
                var a4 = paper1.rect(x, y + dimH * 1, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#4e0c6c", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 4)
                    .click(gridSelL2);

                var text4 = paper1.text(dimW / 4 + x, dimH * 2 + y, 4);
                text4.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text4.data("index", 4);

                var a4cam2 = paper2.rect(x2, y2 + dimH2 * 1, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#4e0c6c", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 4)
                    .click(gridSelL2);

                var text4cam2 = paper2.text(dimW2 / 4 + x2, dimH2 * 2 + y2, 4);
                text4cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text4cam2.data("index", 4);

                var a5 = paper1.rect(x + dimW, y + dimH * 1, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#F2Dbe7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 5)
                    .click(gridSelL2);

                var text5 = paper1.text(dimW / 4 + x + dimW, dimH * 2 + y, 5);
                text5.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text5.data("index", 5);

                var a5cam2 = paper2.rect(x2 + dimW2, y2 + dimH2 * 1, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#4e0c6c", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 5)
                    .click(gridSelL2);

                var text5cam2 = paper2.text(dimW2 / 4 + x2 + dimW2, dimH2 * 2 + y2, 5);
                text5cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text5cam2.data("index", 5);

                var a6 = paper1.rect(x + dimW * 2, y + dimH * 1, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#2eFDb7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 6)
                    .click(gridSelL2);

                var text6 = paper1.text(dimW / 4 + x + dimW * 2, dimH * 2 + y, 6);
                text6.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text6.data("index", 6);

                var a6cam2 = paper2.rect(x2 + dimW2 * 2, y2 + dimH2 * 1, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#2eFDb7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 6)
                    .click(gridSelL2);

                var text6cam2 = paper2.text(dimW2 / 4 + x2 + dimW2 * 2, dimH2 * 2 + y2, 6);
                text6cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text6cam2.data("index", 6);

                // second row
                var a7 = paper1.rect(x, y + dimH * 2, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#Ecc64e", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 7)
                    .click(gridSelL2);
                var text7 = paper1.text(dimW / 4 + x + dimW * 0, dimH * 3 + y, 7);
                text7.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text7.data("index", 7);

                var a7cam2 = paper2.rect(x2, y2 + dimH2 * 2, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#Ecc64e", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 7)
                    .click(gridSelL2);
                var text7cam2 = paper2.text(dimW / 4 + x + dimW * 0, dimH * 3 + y, 7);
                text7cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });

                text7cam2.data("index", 7);

                var a8 = paper1.rect(x + dimW, y + dimH * 2, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#c7Dbe7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 8)
                    .click(gridSelL2);

                var text8 = paper1.text(dimW / 4 + x + dimW, dimH * 3 + y, 8);
                text8.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text8.data("index", 8);

                var a8cam2 = paper2.rect(x2 + dimW2, y2 + dimH2 * 2, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#c7Dbe7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 8)
                    .click(gridSelL2);

                var text8cam2 = paper2.text(dimW2 / 4 + x + dimW2, dimH2 * 3 + y2, 8);
                text8cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text8cam2.data("index", 8);

                var a9 = paper1.rect(x + dimW * 2, y + dimH * 2, dimW, dimH)
                    .attr({visibility: "hidden", fill: "#2eFDb7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 9)
                    .click(gridSelL2);

                var text9 = paper1.text(dimW / 4 + x + dimW * 2, dimH * 3 + y, 9);
                text9.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text9.data("index", 9);

                var a9cam2 = paper2.rect(x2 + dimW2 * 2, y2 + dimH2 * 2, dimW2, dimH2)
                    .attr({visibility: "hidden", fill: "#2eFDb7", stroke: "#000", strokeWidth: 2, "fill-opacity": 0.30})
                    .data("index", 9)
                    .click(gridSelL2);

                var text9cam2 = paper2.text(dimW2 / 4 + x + dimW2 * 2, dimH2 * 3 + y2, 9);
                text9cam2.attr({
                    visibility: "hidden",
                    'font-size': 20
                });
                text9cam2.data("index", 9);

                a1.attr({id: 'paper1'});
                text1.attr({id: 'paper1'});

                a1cam2.attr({id: 'paper2'});
                text1cam2.attr({id: 'paper2'});

                a2.attr({id: 'paper1'});
                text2.attr({id: 'paper1'});

                a2cam2.attr({id: 'paper2'});
                text2cam2.attr({id: 'paper2'});

                a3.attr({id: 'paper1'});
                text3.attr({id: 'paper1'});

                a3cam2.attr({id: 'paper2'});
                text3cam2.attr({id: 'paper2'});

                a4.attr({id: 'paper1'});
                text4.attr({id: 'paper1'});

                a4cam2.attr({id: 'paper2'});
                text4cam2.attr({id: 'paper2'});

                a5.attr({id: 'paper1'});
                text5.attr({id: 'paper1'});

                a5cam2.attr({id: 'paper2'});
                text5cam2.attr({id: 'paper2'});

                a6.attr({id: 'paper1'});
                text6.attr({id: 'paper1'});

                a6cam2.attr({id: 'paper2'});
                text6cam2.attr({id: 'paper2'});

                a7.attr({id: 'paper1'});
                text7.attr({id: 'paper1'});

                a7cam2.attr({id: 'paper2'});
                text7cam2.attr({id: 'paper2'});

                a8.attr({id: 'paper1'});
                text8.attr({id: 'paper1'});

                a8cam2.attr({id: 'paper2'});
                text8cam2.attr({id: 'paper2'});

                a9.attr({id: 'paper1'});
                text9.attr({id: 'paper1'});

                a9cam2.attr({id: 'paper2'});
                text9cam2.attr({id: 'paper2'});

                a1.data("grandparent", block2.data("parent"));
                text1.data("grandparent", block2.data("parent"))
                a1.data("parent", numGrid2);
                text1.data("parent", numGrid2);

                a1cam2.data("grandparent", block2cam2.data("parent"));
                text1cam2.data("grandparent", block2cam2.data("parent"));
                a1cam2.data("parent", numGrid2);
                text1cam2.data("parent", numGrid2);

                a2.data("grandparent", block2.data("parent"));
                text2.data("grandparent", block2.data("parent"))
                a2.data("parent", numGrid2);
                text2.data("parent", numGrid2);

                a2cam2.data("grandparent", block2cam2.data("parent"));
                text2cam2.data("grandparent", block2cam2.data("parent"));
                a2cam2.data("parent", numGrid2);
                text2cam2.data("parent", numGrid2);

                a3.data("grandparent", block2.data("parent"));
                text3.data("grandparent", block2.data("parent"))
                a3.data("parent", numGrid2);
                text3.data("parent", numGrid2);

                a3cam2.data("grandparent", block2cam2.data("parent"));
                text3cam2.data("grandparent", block2cam2.data("parent"));
                a3cam2.data("parent", numGrid2);
                text3cam2.data("parent", numGrid2);

                a4.data("grandparent", block2.data("parent"));
                text4.data("grandparent", block2.data("parent"))
                a4.data("parent", numGrid2);
                text4.data("parent", numGrid2);

                a4cam2.data("grandparent", block2cam2.data("parent"));
                text4cam2.data("grandparent", block2cam2.data("parent"));
                a4cam2.data("parent", numGrid2);
                text4cam2.data("parent", numGrid2);

                a5.data("grandparent", block2.data("parent"));
                text5.data("grandparent", block2.data("parent"))
                a5.data("parent", numGrid2);
                text5.data("parent", numGrid2);

                a5cam2.data("grandparent", block2cam2.data("parent"));
                text5cam2.data("grandparent", block2cam2.data("parent"));
                a5cam2.data("parent", numGrid2);
                text5cam2.data("parent", numGrid2);

                a6.data("grandparent", block2.data("parent"));
                text6.data("grandparent", block2.data("parent"))
                a6.data("parent", numGrid2);
                text6.data("parent", numGrid2);

                a6cam2.data("grandparent", block2cam2.data("parent"));
                text6cam2.data("grandparent", block2cam2.data("parent"));
                a6cam2.data("parent", numGrid2);
                text6cam2.data("parent", numGrid2);

                a7.data("grandparent", block2.data("parent"));
                text7.data("grandparent", block2.data("parent"))
                a7.data("parent", numGrid2);
                text7.data("parent", numGrid2);

                a7cam2.data("grandparent", block2cam2.data("parent"));
                text7cam2.data("grandparent", block2cam2.data("parent"));
                a7cam2.data("parent", numGrid2);
                text7cam2.data("parent", numGrid2);

                a8.data("grandparent", block2.data("parent"));
                text8.data("grandparent", block2.data("parent"))
                a8.data("parent", numGrid2);
                text8.data("parent", numGrid2);

                a8cam2.data("grandparent", block2cam2.data("parent"));
                text8cam2.data("grandparent", block2cam2.data("parent"));
                a8cam2.data("parent", numGrid2);
                text8cam2.data("parent", numGrid2);

                a9.data("grandparent", block2.data("parent"));
                text9.data("grandparent", block2.data("parent"))
                a9.data("parent", numGrid2);
                text9.data("parent", numGrid2);

                a9cam2.data("grandparent", block2cam2.data("parent"));
                text9cam2.data("grandparent", block2cam2.data("parent"));
                a9cam2.data("parent", numGrid2);
                text9cam2.data("parent", numGrid2);

                gLevel1.append(block2);
                gLevel1.append(text);

                gLevel1cam2.append(block2cam2);
                gLevel1cam2.append(textcam2);

                gLevel2cam2.append(a1cam2);
                gLevel2cam2.append(text1cam2);

                gLevel2cam2.append(a2cam2);
                gLevel2cam2.append(text2cam2);

                gLevel2cam2.append(a3cam2);
                gLevel2cam2.append(text3cam2);

                gLevel2cam2.append(a4cam2);
                gLevel2cam2.append(text4cam2);

                gLevel2cam2.append(a5cam2);
                gLevel2cam2.append(text5cam2);

                gLevel2cam2.append(a6cam2);
                gLevel2cam2.append(text6cam2);

                gLevel2cam2.append(a7cam2);
                gLevel2cam2.append(text7cam2);

                gLevel2cam2.append(a8cam2);
                gLevel2cam2.append(text8cam2);

                gLevel2cam2.append(a9cam2);
                gLevel2cam2.append(text9cam2);

                gLevel2.append(a1);
                gLevel2.append(text1);

                gLevel2.append(a2);
                gLevel2.append(text2);

                gLevel2.append(a3);
                gLevel2.append(text3);

                gLevel2.append(a4);
                gLevel2.append(text4);

                gLevel2.append(a5);
                gLevel2.append(text5);

                gLevel2.append(a6);
                gLevel2.append(text6);

                gLevel2.append(a7);
                gLevel2.append(text7);

                gLevel2.append(a8);
                gLevel2.append(text8);

                gLevel2.append(a9);
                gLevel2.append(text9);

                text1.click(gridSelL2);
                text2.click(gridSelL2);
                text3.click(gridSelL2);
                text4.click(gridSelL2);
                text5.click(gridSelL2);
                text6.click(gridSelL2);
                text7.click(gridSelL2);
                text8.click(gridSelL2);
                text9.click(gridSelL2);

                text1cam2.click(gridSelL2);
                text2cam2.click(gridSelL2);
                text3cam2.click(gridSelL2);
                text4cam2.click(gridSelL2);
                text5cam2.click(gridSelL2);
                text6cam2.click(gridSelL2);
                text7cam2.click(gridSelL2);
                text8cam2.click(gridSelL2);
                text9cam2.click(gridSelL2);

            }
        }
    }

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

        var bbox = panelCam1.getBBox();
        var ratioW1 =this.app.getCam1W()/bbox.width;
        var ratioH1 =this.app.getCam1H()/bbox.height;

        var bbox2 = panelCam2.getBBox();
        var ratioW2 =this.app.getCam2W()/bbox2.width;
        var ratioH2 =this.app.getCam2H()/bbox2.height;

        gAll.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);
        gLevel1.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);
        gLevel2.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);
        line1.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);
        circle1.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);

        gAllcam2.animate({ transform:"r0," +bbox2.cx + ',' + bbox2.cy + "s"+ratioW2+"," +ratioH2+"," + bbox2.cx + "," + bbox2.cy}, 2000);
        gLevel1cam2.animate({ transform:"r0," +bbox2.cx + ',' + bbox2.cy + "s"+ratioW2+"," +ratioH2+"," + bbox2.cx + "," + bbox2.cy}, 2000);
        gLevel2cam2.animate({ transform:"r0," +bbox2.cx + ',' + bbox2.cy + "s"+ratioW2+"," +ratioH2+"," + bbox2.cx + "," + bbox2.cy}, 2000);
        line2.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);
        circle2.animate({ transform: "r0," +bbox.cx + ',' + bbox.cy + "s"+ratioW1+"," +ratioH1+"," + bbox.cx + "," + bbox.cy}, 2000);

        panelCam1.style.width=this.app.getCam1W()+ 'px';
        panelCam1.style.height=this.app.getCam1H()+ 'px';

        panelCam2.style.width=this.app.getCam2W()+ 'px';
        panelCam2.style.height=this.app.getCam2H()+ 'px';

        panel1.style.top=-this.app.getCam1H()+ 'px';
        panel1.style.left=0+ 'px';

        panel2.style.top=-this.app.getCam2H()+ 'px';
        panel2.style.left=0+ 'px';

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