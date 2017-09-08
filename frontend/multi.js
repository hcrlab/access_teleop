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



    var camera1 = document.getElementById("camera1");
    var camera2 = document.getElementById("camera2");
    var camera3 = document.getElementById("camera3");

    var titleWeb = document.getElementById("title");
    var titleWebH = titleWeb.offsetHeight;

    var btnTop = document.getElementById("top");
    var btnTopH = btnTop.offsetHeight;

    var btnUpH = camera1.offsetHeight + titleWebH + btnTopH;
    var upBTN = document.createElement("button");
    upBTN.innerHTML = "<img src=\"img/upArrow.png\">";
    upBTN.id = 'up';
    upBTN.title = '\u2191Up\u2191';
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(upBTN);

    var downBTN = document.createElement("button");
    downBTN.innerHTML = "<img src=\"img/downArrow.png\">";
    downBTN.id = 'down';
    downBTN.title = '\u2193Down\u2193';
    downBTN.className = "button";
    downBTN.style.left = (camera1.offsetWidth - 95) / 2 + "px";
    downBTN.style.top = (btnUpH - titleWebH / 1.5) / 1 + "px";
    body.appendChild(downBTN);

    var leftBTN = document.createElement("button");
    leftBTN.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN.id = 'left';
    leftBTN.title = '\u2190Left\u2190';
    leftBTN.className = "button";
    leftBTN.style.left = 0 + "px";
    leftBTN.style.top = (btnUpH) / 2 + "px";
    leftBTN.style.width = 55 + "px";
    leftBTN.style.height = 80 + "px";
    body.appendChild(leftBTN);

    var rotateRightBTN = document.createElement("button");
    rotateRightBTN.innerHTML = "<img src=\"img/rotateClockwiseRight.png\">";
    rotateRightBTN.id = 'rotateRight';
    rotateRightBTN.title = '\u21BBRotate Right\u21BB';
    rotateRightBTN.className = "button";
    rotateRightBTN.style.left = 55 + "px";
    rotateRightBTN.style.top = (btnUpH) / 2 + "px";
    rotateRightBTN.style.height = 80 + "px";
    body.appendChild(rotateRightBTN);

    var rightBTN = document.createElement("button");
    rightBTN.innerHTML = "<img src=\"img/greenRightArrow.png\">";
    rightBTN.id = 'right';
    rightBTN.title = '\u2192Right\u2192';
    rightBTN.className = "button";
    rightBTN.style.width = 55 + "px";
    rightBTN.style.height = 80 + "px";
    rightBTN.style.left = camera1.offsetWidth - titleWebH - 55 / 4 + "px";
    rightBTN.style.top = (btnUpH) / 2 + "px";
    body.appendChild(rightBTN);


    var rotateLeftBTN = document.createElement("button");
    rotateLeftBTN.innerHTML = "<img src=\"img/rotateAntiClockwiseLeft.png\">";
    rotateLeftBTN.id = 'rotateLeft';
    rotateLeftBTN.title = '\u21BARotate Left\u21BA';
    rotateLeftBTN.className = "button";
    rotateLeftBTN.style.width = 65 + "px";
    rotateLeftBTN.style.height = 80 + "px";
    rotateLeftBTN.style.left = camera1.offsetWidth - titleWebH - 75 + "px";
    rotateLeftBTN.style.top = (btnUpH) / 2 + "px";
    rotateLeftBTN.style.height = 80 + "px";
    body.appendChild(rotateLeftBTN);

    upBTN.addEventListener("click", function () {
        alert("Go Up");
    });

    downBTN.addEventListener("click", function () {
        alert("Go Down");
    });

    leftBTN.addEventListener("click", function () {
        alert("Go Left");
    });

    rightBTN.addEventListener("click", function () {
        alert("Go Right");
    });

    rotateRightBTN.addEventListener("click", function () {
        alert("Rotate Right");
    });

    $("#up").hide();
    $("#down").hide();
    $("#left").hide()
    $("#right").hide();
    $("#rotateRight").hide()
    $("#rotateLeft").hide();

    camera1.addEventListener("mouseover", showButtons);
    camera1.addEventListener("mouseout", hideButtons);
    camera2.addEventListener("mouseover", showButtons);
    camera2.addEventListener("mouseout", hideButtons);
    camera3.addEventListener("mouseover", showButtons);
    camera3.addEventListener("mouseout", hideButtons);

    function showButtons() {

        switch (this.id) {
            case "camera1":
                $("#cam1").css("color", "yellow");

                upBTN.style.left = (camera1.offsetWidth - 95) / 2 + "px";
                upBTN.style.top = (btnUpH) / 4 + "px";
                $("#up").show();
                $("#down").show();
                $("#left").show();
                $("#right").show();
                $("#rotateRight").show()
                $("#rotateLeft").show();
                break;
            case "camera2":
                $("#cam2").css("color", "yellow");
                //$("#up").show();
                //  $("#down").show();
                //  $("#left").show();
                //  $("#right").show();
                break;
            case "camera3":
                $("#title3").css("color", "yellow");
                break;
        }
    }

    function hideButtons() {

        switch (this.id) {
            case "camera1":
                $("#title1").css("color", "blue");
                $("#up").hide();
                $("#down").hide();
                $("#left").hide();
                $("#right").hide()
                $("#rotateRight").hide()
                $("#rotateLeft").hide();
                break;
            case "camera2":
                $("#title2").css("color", "blue");
                break;
            case "camera3":
                $("#title3").css("color", "white");
                break;
        }
    }

    init_flag = false;

}


