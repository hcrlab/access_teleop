
$(document).ready(function(){

var up1BTN = document.getElementById("upBtn1");
var up2BTN = document.getElementById("upBtn2");
var right1BTN = document.getElementById("rightBtn1");
var right2BTN = document.getElementById("rightBtn2");
var down1BTN = document.getElementById("downBtn1");
var down2BTN = document.getElementById("downBtn2");
var left1BTN = document.getElementById("leftBtn1");
var left2BTN = document.getElementById("leftBtn2");
var rotateLeft1BTN = document.getElementById("rotateLeftBtn1");
var rotateLeft2BTN = document.getElementById("rotateLeftBtn2");
var rotateRight1BTN = document.getElementById("rotateRightBtn1");
var rotateRight2BTN = document.getElementById("rotateRightBtn2");


    $(window).resize(function () {
        var camera1 = document.getElementById("camera1");
        var camera2 = document.getElementById("camera2");
        var camera3 = document.getElementById("camera3");
        $docWidth =  $("#camera1").width();
        $elWidth = $("#controls1").width();
        $offset = ($docWidth - $elWidth) / 2;
        $("#controls1").css("top", - $("#camera1").height()- 7+ "px");
        $("#upBtn1").css("left",  $("#camera1").width()*0.5 + "px");
        $("#upBtn1").css("top", 0+ "px");
        var nav = $('#upBtn1');
        if (nav.length) {
         //   var contentNav = nav.offset().top;

        alert("Top: " +  $("#upBtn1").position().top + " Left: " +  $("#upBtn1").position().left);
        var elem = $("#upBtn1");
    }
        // ();       alert('x: ' + x + ' y: ' + y);
    });
    });