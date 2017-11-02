/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */

function init() {
    this.app = new App();
    var self = this;

    var arm_div = document.querySelectorAll('.js_arm_div');
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
    controls1.style.left=topBTN.left+"px";
    controls1.style.top=-this.app.dimCam1Height- 7+"px";


    var controls2= document.getElementById("controls2");
    controls2.style.left=sideBTN.left+"px";
    controls2.style.top=-this.app.dimCam2Height- 7+"px";
    var upBTN1 =document.getElementById("upBtn1");
    upBTN1.innerHTML="<img src=\"img/blueUpArrow.png\">";
    upBTN1.id='up';
    upBTN1.title='\u2191Up\u2191';
    upBTN1.style.left=cam1W*0.5- upBTN1.clientWidth/2+"px";
    upBTN1.style.top=0+"px";

    var downBTN1 = document.getElementById("downBtn1");
    downBTN1.innerHTML = "<img src=\"img/blueDownArrow.png\">";
    downBTN1.id='down';
    downBTN1.title='\u2191Down\u2191';
    downBTN1.style.left=cam1W*0.5- downBTN1.clientWidth/2+"px";
    downBTN1.style.top=cam1H- downBTN1.clientHeight+"px";

    var leftBTN1 =  document.getElementById("leftBtn1");
    leftBTN1.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN1.id='left';
    leftBTN1.title='\u2190Left\u2190[';
    leftBTN1.style.left=0+"px";
    leftBTN1.style.top=cam1H/2 -leftBTN1.clientHeight/2+"px";

    var rightBTN1 =  document.getElementById("rightBtn1");
    rightBTN1.innerHTML = "<img src=\"img/greenRightArrow.png\">";
    rightBTN1.id='right';
    rightBTN1.title='\u2192Right\u2192';
    rightBTN1.style.left=cam1W-rightBTN1.clientWidth+"px";
    rightBTN1.style.top=cam1H/2 -rightBTN1.clientHeight/2+"px";

    var rotateRightBtn1 =  document.getElementById("rotateRightBtn1");
    rotateRightBtn1.innerHTML = "<img src=\"img/rotateClockwiseRight.png\">";
    rotateRightBtn1.id = 'rotateRight';
    rotateRightBtn1.style.height = 80 + "px";
    rotateRightBtn1.title = '\u21BBRotate Right\u21BB';
    rotateRightBtn1.style.left = cam1W-rightBTN1.clientWidth-rotateRightBtn1.clientWidth+ "px";
    rotateRightBtn1.style.top =cam1H/2 -rotateRightBtn1.clientHeight/2+ "px";

    var rotateLeftBtn1 =  document.getElementById("rotateLeftBtn1");
    rotateLeftBtn1.innerHTML = "<img src=\"img/rotateAntiClockwiseLeft.png\">";
    rotateLeftBtn1.id = 'rotateLeft';
    rotateLeftBtn1.title = '\u21BARotate Left\u21BA';
    rotateLeftBtn1.style.left =leftBTN1.clientWidth+ "px";
    rotateLeftBtn1.style.top =cam1H/2 -rotateLeftBtn1.clientHeight/2+ "px";


    var upBTN2 =  document.getElementById("upBtn2");
    upBTN2.innerHTML="<img src=\"img/redUpArrow.png\">";
    upBTN2.id='up2';
    upBTN2.title='\u2191Up\u2191';
    upBTN2.style.left=cam2W*0.5- upBTN2.clientWidth/2+"px";
    upBTN2.style.top=0+ "px";

    var downBTN2 =  document.getElementById("downBtn2");
    downBTN2.innerHTML = "<img src=\"img/redDownArrow.png\">";
    downBTN2.id='down2';
    downBTN2.title='\u2191Down\u2191';
    downBTN2.className = "button";
    downBTN2.style.left = cam2W*0.5- upBTN2.clientWidth/2+"px";
    downBTN2.style.top= cam2H- downBTN2.clientHeight+"px";

    var leftBTN2 =  document.getElementById("leftBtn2");
    leftBTN2.innerHTML = "<img src=\"img/blueLeftArrow.png\">";
    leftBTN2.id='left2';
    leftBTN2.title='\u2191Left\u2191';
    leftBTN2.style.left=0 + "px";
    leftBTN2.style.top=cam2H/2 -leftBTN2.clientHeight/2+ "px";

    var rightBTN2 =  document.getElementById("rightBtn2");
    rightBTN2.innerHTML = "<img src=\"img/blueRightArrow.png\">";
    rightBTN2.id='right2';
    rightBTN2.title='\u2191Right\u2191';
    rightBTN2.className = "button";
    rightBTN2.style.left=cam2W-rightBTN2.clientWidth+ "px";
    rightBTN2.style.top= cam2H/2 -rightBTN2.clientHeight/2+ "px";

    var rotaterightBTN2 =  document.getElementById("rotateRightBtn2");
    rotaterightBTN2.innerHTML = "<img src=\"img/greenRotateClockRight.png\">";
    rotaterightBTN2.id = 'rotateRight2';
    rotaterightBTN2.title = '\u21BBRotate Right\u21BB';
    rotaterightBTN2.className = "button";
    rotaterightBTN2.style.left =cam2W-rightBTN2.clientWidth-rotaterightBTN2.clientWidth+ "px";
    rotaterightBTN2.style.top =cam2H/2 -rotaterightBTN2.clientHeight/2+ "px";
    rotaterightBTN2.style.height = 80 + "px";
    var rotateleftBTN2 =  document.getElementById("rotateLeftBtn2");
    rotateleftBTN2.innerHTML = "<img src=\"img/greenRotateAntiClockLeft.png\">";
    rotateleftBTN2.id = 'rotateLeft2';
    rotateleftBTN2.title = '\u21BARotate Left\u21BA';
    rotateleftBTN2.className = "button";
    rotateleftBTN2.style.width = 65 + "px";
    rotateleftBTN2.style.height = 80 + "px";
    rotateleftBTN2.style.left =  leftBTN2.clientWidth+ "px";
    rotateleftBTN2.style.top = cam2H/2 -rotateleftBTN2.clientHeight/2+ "px";
    rotateleftBTN2.style.height = 80 + "px";


    topBTN.addEventListener("click", setButtonDisplay);
    sideBTN.addEventListener("click", setButtonDisplay);
    headBTN.addEventListener("click", setButtonDisplay);

    upBTN1.addEventListener ("click", pubMoveMsg);
    downBTN1.addEventListener ("click", pubMoveMsg);
    leftBTN1.addEventListener ("click", pubMoveMsg);
    rightBTN1.addEventListener ("click", pubMoveMsg);
    rotateRightBtn1.addEventListener ("click", pubMoveMsg);
    rotateLeftBtn1.addEventListener ("click", pubMoveMsg);

    upBTN2.addEventListener ("click", pubMoveMsg);
    downBTN2.addEventListener ("click", pubMoveMsg);
    leftBTN2.addEventListener ("click", pubMoveMsg);
    rightBTN2.addEventListener ("click", pubMoveMsg);
    rotaterightBTN2.addEventListener ("click", pubMoveMsg);
    rotateleftBTN2.addEventListener ("click", pubMoveMsg);
    function setButtonDisplay() {
        var btnSel;
        var expr = this.id;
        switch (expr) {
            case 'topBTN':
                btnSel = "topBTN";
                break;
            case 'sideBTN':
                btnSel = "sideBTN";
                break;
            case 'headBTN':
                btnSel = "headBTN";
                break;
        }
    }

    var cam1 = document.getElementById("camera1");
    var cam2 = document.getElementById("camera2");

    var topBTN = document.getElementById("topBTN");

    var resizeOverlay = _.debounce(function() {
        /*         // the code below should be working instead of location.reload*/
        var ratio =this.app.getCam1W()/this.app.getCam1WO();
        //alert(leftBTN1.style.left);
        console.log("original ="+this.app.getCam1WO());
        controls1.style.transform='scale('+ratio+','+ratio+') translate('+-this.app.getCam1WO()/2*ratio+'px, 0px)';
        controls2.style.transform='scale('+ratio+','+ratio+') translate('+-this.app.getCam2WO()/2*ratio+'px, 0px)';
        controls1.style.top=-cam1.clientHeight+ 'px';
        controls2.style.left=-sideBTN.left+ 'px';
        controls2.style.top=-cam2.clientHeight+ 'px';
        console.log("new ="+this.app.getCam1W());
        //       console.log("ratio ="+ratio);
        // location.reload();
    }, 400);

    window.addEventListener('resize', resizeOverlay);

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

}