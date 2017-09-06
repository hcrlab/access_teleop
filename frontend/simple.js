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

    var upBTN = document.createElement("button");
    upBTN.innerHTML="<img src=\"img/upArrow.png\">";
    upBTN.id='up';
    upBTN.title='\u2191Up\u2191';
    upBTN.style.left=(elmnt.offsetWidth - 95)/2 + "px";
    upBTN.style.top= (elmnt.offsetHeight)/4 + "px";
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(upBTN);

    var downBTN = document.createElement("button");
    downBTN.innerHTML = "<img src=\"img/downArrow.png\">";
    downBTN.id='down';
    downBTN.title='\u2191Down\u2191';
    downBTN.className = "button";
    downBTN.style.left=(elmnt.offsetWidth - 95)/2 + "px";
    downBTN.style.top= (elmnt.offsetHeight + 32)/1 + "px";
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(downBTN);


    var leftBTN = document.createElement("button");
    leftBTN.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN.id='left';
    leftBTN.title='\u2191Left\u2191';
    leftBTN.className = "button";
    leftBTN.style.left=0 + "px";
    leftBTN.style.top= (elmnt.offsetHeight + 32)/2 + "px";
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(leftBTN);


    var rightBTN = document.createElement("button");
    rightBTN.innerHTML = "<img src=\"img/greenRightArrow.png\">";
    rightBTN.id='right';
    rightBTN.title='\u2191Right\u2191';
    rightBTN.className = "button";
    rightBTN.style.left=(elmnt.offsetWidth - 57) + "px";
    rightBTN.style.top= (elmnt.offsetHeight + 32)/2 + "px";
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(rightBTN);

    upBTN.addEventListener ("click", function() {
        alert("Go Up");
    });

    downBTN.addEventListener ("click", function() {
        alert("Go Down");
    });

    leftBTN.addEventListener ("click", function() {
        alert("Go Left");
    });

    rightBTN.addEventListener ("click", function() {
        alert("Go Right");
    });


}