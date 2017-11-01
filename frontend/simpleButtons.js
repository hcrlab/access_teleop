/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */




function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    app.ros.on('connection', function () {
        console.log("We are connected!");
    });

    var cam1Con = document.getElementById("cam1-container");
    var cam2Con = document.getElementById("cam2-container");
    var cam1 = document.getElementById("camera1");

    var cam1position = cam1Con.getBoundingClientRect();
    var cam2position = cam2Con.getBoundingClientRect();

    var topBTN = document.getElementById("topBTN");
    var sideBTN = document.getElementById("sideBTN");
    var headBTN = document.getElementById("headBTN");
    var cam1W=this.app.dimCam1Width;
    var cam1H = this.app.dimCam1Height;
    var cam2W=this.app.dimCam2Width;
    var cam2H = this.app.dimCam2Height;
    var cam3W=this.app.dimCam3Width;
    var cam3H = this.app.dimCam3Height;

    var controls1 = document.getElementById("controls1");
 //   controls1.style.left=topBTN.left+"px";
//    controls1.style.top=-this.app.dimCam1Height- 7+"px";

    var controls2= document.getElementById("controls2");
 //   controls2.style.left=sideBTN.left+"px";
 //   controls2.style.top=-this.app.dimCam2Height- 7+"px";

    var panel1= document.getElementById("controls1");
    panel1.style.top=-this.app.dimCam1Height+ 'px';
    panel1.style.left=0+ 'px';

    var panel2= document.getElementById("controls2");
    panel2.style.top=-this.app.dimCam2Height+ 'px';
    panel2.style.left=0+ 'px';

    var panelCam1= document.getElementById("btnCam1");
    panelCam1.style.width=this.app.dimCam1Width+ 'px';
    panelCam1.style.height=this.app.dimCam1Height+ 'px';

    var panelCam2= document.getElementById("btnCam2");
    panelCam2.style.width=this.app.dimCam2Width+ 'px';
    panelCam2.style.height=this.app.dimCam2Height+ 'px';


    var upBTN1 =document.getElementById("upBtn1");
    upBTN1.innerHTML="<img src=\"img/blueUpArrow.png\">";
    upBTN1.id='up';
    upBTN1.title='\u2191Up\u2191';
    upBTN1.style.left=(cameraWidth - 95)/2 + "px";
    upBTN1.style.top= cameraHeight/4  + 10 + "px";
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(upBTN1);

    var downBTN1 = document.createElement("button");
    downBTN1.innerHTML = "<img src=\"img/blueDownArrow.png\">";
    downBTN1.id='down';
    downBTN1.title='\u2191Down\u2191';
    downBTN1.className = "button";
    downBTN1.style.left=(cameraWidth - 95)/2 + "px";
    downBTN1.style.top= (cameraHeight- (-50))+ "px";
    body.appendChild(downBTN1);

    var leftBTN1 = document.createElement("button");
    leftBTN1.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN1.id='left';
    leftBTN1.title='\u2190Left\u2190[';
    leftBTN1.className = "button";
    leftBTN1.style.left=0 + "px";
    leftBTN1.style.top=(cameraHeight- (-90))/2 + "px";
    body.appendChild(leftBTN1);

    var rightBTN1 = document.createElement("button");
    rightBTN1.innerHTML = "<img src=\"img/greenRightArrow.png\">";
    rightBTN1.id='right';
    rightBTN1.title='\u2192Right\u2192';
    rightBTN1.className = "button";
    rightBTN1.style.left=(cameraWidth -52)+ "px";
    rightBTN1.style.top= (cameraHeight- (-90))/2 + "px";
    body.appendChild(rightBTN1);

    var rotaterightBTN1 = document.createElement("button");
    rotaterightBTN1.innerHTML = "<img src=\"img/rotateClockwiseRight.png\">";
    rotaterightBTN1.id = 'rotateRight';
    rotaterightBTN1.title = '\u21BBRotate Right\u21BB';
    rotaterightBTN1.className = "button";
    rotaterightBTN1.style.left =(cameraWidth -118)+ "px";
    rotaterightBTN1.style.top =(cameraHeight- (-90))/2 + "px";
    rotaterightBTN1.style.height = 80 + "px";
    body.appendChild(rotaterightBTN1);

    var rotateleftBTN1 = document.createElement("button");
    rotateleftBTN1.innerHTML = "<img src=\"img/rotateAntiClockwiseLeft.png\">";
    rotateleftBTN1.id = 'rotateLeft';
    rotateleftBTN1.title = '\u21BARotate Left\u21BA';
    rotateleftBTN1.className = "button";
    rotateleftBTN1.style.width = 65 + "px";
    rotateleftBTN1.style.height = 80 + "px";
    rotateleftBTN1.style.left =  55 + "px";
    rotateleftBTN1.style.top = (cameraHeight- (-90))/2 + "px";
    rotateleftBTN1.style.height = 80 + "px";
    body.appendChild(rotateleftBTN1);

    var upBTN2 = document.createElement("button");
    upBTN2.innerHTML="<img src=\"img/redUpArrow.png\">";
    upBTN2.id='up2';
    upBTN2.title='\u2191Up\u2191';
    upBTN2.style.left=(cameraWidth - 100)*2 + "px";
    upBTN2.style.top= cameraHeight/4  + 10 + "px";
    body.appendChild(upBTN2);

    var downBTN2 = document.createElement("button");
    downBTN2.innerHTML = "<img src=\"img/redDownArrow.png\">";
    downBTN2.id='down2';
    downBTN2.title='\u2191Down\u2191';
    downBTN2.className = "button";
    downBTN2.style.left=(cameraWidth - 100)*2 + "px";
    downBTN2.style.top= (cameraHeight- (-50))+ "px";
    body.appendChild(downBTN2);

    var leftBTN2 = document.createElement("button");
    leftBTN2.innerHTML = "<img src=\"img/blueLeftArrow.png\">";
    leftBTN2.id='left2';
    leftBTN2.title='\u2191Left\u2191';
    leftBTN2.className = "button";
    leftBTN2.style.left=(cameraWidth- (-10)) + "px";
    leftBTN2.style.top=(cameraHeight- (-90))/2 + "px";
    body.appendChild(leftBTN2);

    var rightBTN2 = document.createElement("button");
    rightBTN2.innerHTML = "<img src=\"img/blueRightArrow.png\">";
    rightBTN2.id='right2';
    rightBTN2.title='\u2191Right\u2191';
    rightBTN2.className = "button";
    rightBTN2.style.left=(cameraWidth*2 - (48)) + "px";
    rightBTN2.style.top= (cameraHeight- (-90))/2 + "px";
    body.appendChild(rightBTN2);

    var rotaterightBTN2 = document.createElement("button");
    rotaterightBTN2.innerHTML = "<img src=\"img/greenRotateClockRight.png\">";
    rotaterightBTN2.id = 'rotateRight2';
    rotaterightBTN2.title = '\u21BBRotate Right\u21BB';
    rotaterightBTN2.className = "button";
    rotaterightBTN2.style.left =(cameraWidth*2- (115)) + "px";
    rotaterightBTN2.style.top =(cameraHeight- (-90))/2 + "px";
    rotaterightBTN2.style.height = 80 + "px";
    body.appendChild(rotaterightBTN2);

    var rotateleftBTN2 = document.createElement("button");
    rotateleftBTN2.innerHTML = "<img src=\"img/greenRotateAntiClockLeft.png\">";
    rotateleftBTN2.id = 'rotateLeft2';
    rotateleftBTN2.title = '\u21BARotate Left\u21BA';
    rotateleftBTN2.className = "button";
    rotateleftBTN2.style.width = 65 + "px";
    rotateleftBTN2.style.height = 80 + "px";
    rotateleftBTN2.style.left =  (cameraWidth- (-65)) + "px";
    rotateleftBTN2.style.top = (cameraHeight- (-90))/2 + "px";
    rotateleftBTN2.style.height = 80 + "px";
    body.appendChild(rotateleftBTN2);

    upBTN1.addEventListener ("click", pubMoveMsg);
    downBTN1.addEventListener ("click", pubMoveMsg);
    leftBTN1.addEventListener ("click", pubMoveMsg);
    rightBTN1.addEventListener ("click", pubMoveMsg);
    rotaterightBTN1.addEventListener ("click", pubMoveMsg);
    rotateleftBTN1.addEventListener ("click", pubMoveMsg);
    upBTN2.addEventListener ("click", pubMoveMsg);
    downBTN2.addEventListener ("click", pubMoveMsg);
    leftBTN2.addEventListener ("click", pubMoveMsg);
    rightBTN2.addEventListener ("click", pubMoveMsg);
    rotaterightBTN2.addEventListener ("click", pubMoveMsg);
    rotateleftBTN2.addEventListener ("click", pubMoveMsg);


    var cam1 = document.getElementById("camera1");
    var cam2 = document.getElementById("camera2");

    var topBTN = document.getElementById("topBTN");
    var resizeOverlay = _.debounce(function() {
        var ratio1 =this.app.getCam1W()/this.app.getCam1WO();
        var ratio2 =this.app.getCam2W()/this.app.getCam2WO();
        panelCam1.style.transform='scale('+ratio1+','+ratio1+')';
        panelCam2.style.transform='scale('+ratio2+','+ratio2+')';
        var  leftMargin1= ((this.app.getCam1W() * ratio1) - this.app.getCam1W()) / 2;
        var  topMargin1= ((this.app.getCam1H() * ratio1) - this.app.getCam1H()) / 2;
        panel1.style.top=-this.app.getCam2H()+topMargin1 + 'px';
        panel1.style.left=leftMargin1   + 'px';

        panelCam1.style.width=this.app.getCam1W()+ 'px';
        panelCam1.style.height=this.app.getCam1H()+ 'px';
        var leftMargin2  = ((this.app.getCam2W() * ratio2) - this.app.getCam2W()) / 2;
        var topMargin2 = ((this.app.getCam2H() * ratio2) - this.app.getCam2H()) / 2;
        panel2.style.top=-this.app.getCam2H()+topMargin2 + 'px';
        panel2.style.left=leftMargin2+ 'px';

        panelCam2.style.width=this.app.getCam2W()+ 'px';
        panelCam2.style.height=this.app.getCam2H()+ 'px';

    }, 400);

    window.addEventListener('resize', resizeOverlay);



    /*
        var resizeOverlay = _.debounce(function() {

controls1.style.top=-cam1.clientHeight+ 'px';
            controls2.style.left=-sideBTN.left+ 'px';
            controls2.style.top=-cam2.clientHeight+ 'px';
            console.log("new ="+this.app.getCam1W());
            //       console.log("ratio ="+ratio);
            // location.reload();
        }, 400);

        window.addEventListener('resize', resizeOverlay);
    */

    function pubMoveMsg() {
        var comSel;
        var expr = this.id;
        switch (expr) {
            case 'up':
                comSel = "Go Up 1";
                break;
            case 'up2':
                comSel = "Go Up 2";
                break;
            case 'down':
                comSel = "Go Down 1";
                break;
            case 'down2':
                comSel = "Go Down 2";
                break;
            case 'left':
                comSel = "Go Left 1";
                break;
            case 'left2':
                comSel = "Go Left 2";
                break;
            case 'right':
                comSel = "Go Right 1";
                break;
            case 'right2':
                comSel = "Go Right 2";
                break;
            case 'rotateRight':
                comSel = "Go Rotate Right 1";
                break;
            case 'rotateRight2':
                comSel = "Go Rotate Right 2";
                break;
            case 'rotateLeft':
                comSel = "Go Rotate Left";
                break;
            case 'rotateLeft2':
                comSel = "Go Rotate Left 2";
                break;
        }

        document.getElementById("cmdReceived").innerHTML = comSel;
    }

    document.getElementById("cmdReceived").innerHTML = comSel;
}
