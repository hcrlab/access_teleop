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

    var elmnt = document.getElementById("camera1");

    var cameraWidth = "342";
    var cameraHeight = "342";

    var upBTN1 = document.createElement("button");
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
    downBTN1.title='\u2193Down\u2193';
    downBTN1.className = "button";
    downBTN1.style.left=(cameraWidth - 95)/2 + "px";
    downBTN1.style.top= (cameraHeight- (-50))+ "px";
    body.appendChild(downBTN1);

    var leftBTN1 = document.createElement("button");
    leftBTN1.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN1.id='left';
    leftBTN1.title='\u2190Left\u2190';
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
    rotaterightBTN1.title = '\u21B6Rotate Right\u21B6';
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
    downBTN2.title='\u2193Down\u2193';
    downBTN2.className = "button";
    downBTN2.style.left=(cameraWidth - 100)*2 + "px";
    downBTN2.style.top= (cameraHeight- (-50))+ "px";
    body.appendChild(downBTN2);

    var leftBTN2 = document.createElement("button");
    leftBTN2.innerHTML = "<img src=\"img/blueLeftArrow.png\">";
    leftBTN2.id='left2';
    leftBTN2.title='\u2190Left\u2190';
    leftBTN2.className = "button";
    leftBTN2.style.left=(cameraWidth- (-10)) + "px";
    leftBTN2.style.top=(cameraHeight- (-90))/2 + "px";
    body.appendChild(leftBTN2);

    var rightBTN2 = document.createElement("button");
    rightBTN2.innerHTML = "<img src=\"img/blueRightArrow.png\">";
    rightBTN2.id='right2';
    rightBTN2.title='\u2192Right\u2192';
    rightBTN2.className = "button";
    rightBTN2.style.left=(cameraWidth*2 - (48)) + "px";
    rightBTN2.style.top= (cameraHeight- (-90))/2 + "px";
    body.appendChild(rightBTN2);

    var rotaterightBTN2 = document.createElement("button");
    rotaterightBTN2.innerHTML = "<img src=\"img/greenRotateClockRight.png\">";
    rotaterightBTN2.id = 'rotateRight2';
    rotaterightBTN2.title = '\u21B6Rotate Right\u21B6';
    rotaterightBTN2.className = "button";
    rotaterightBTN2.style.left =(cameraWidth*2- (120)) + "px";
    rotaterightBTN2.style.top =(cameraHeight- (-90))/2 + "px";
    rotaterightBTN2.style.height = 80 + "px";
    body.appendChild(rotaterightBTN2);

    var rotateleftBTN2 = document.createElement("button");
    rotateleftBTN2.innerHTML = "<img src=\"img/greenRotateAntiClockLeft.png\">";
    rotateleftBTN2.id = 'rotateLeft2';
    rotateleftBTN2.title = '\u21BARotate Left\u21BA';
    rotateleftBTN2.className = "button";
    rotateleftBTN2.style.left =  (cameraWidth- (-65)) + "px";
    rotateleftBTN2.style.top = (cameraHeight- (-90))/2 + "px";
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




function pubMoveMsg() {
    var comSel;
    var expr = this.id;
    switch (expr) {
      case 'up':
        comSel="Go Up 1";
        break;
      case 'up2':
        comSel="Go Up 2";
        break;
      case 'down':
        comSel="Go Down 1";
        break;
      case 'down2':
        comSel="Go Down 2";
        break;
      case 'left':
        comSel="Go Left 1";
        break;
      case 'left2':
        comSel="Go Left 2";
        break;
      case 'right':
        comSel="Go Right 1";
        break;
      case 'right2':
        comSel="Go Right 2";
        break;
      case 'rotateRight':
        comSel="Go Rotate Right 1";
        break;
      case 'rotateRight2':
        comSel="Go Rotate Right 2";
        break;
      case 'rotateLeft':
        comSel="Go Rotate Left";
        break;
      case 'rotateLeft2':
        comSel="Go Rotate Left 2";
        break;
    }

    document.getElementById("cmdReceived").innerHTML = comSel;
}
}