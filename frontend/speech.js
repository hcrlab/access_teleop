
$(document).ready(function(){
try {
  var SpeechRecognition = window.SpeechRecognition ||
                          window.webkitSpeechRecognition ||
                          window.mozSpeechRecognition ||
                          window.oSpeechRecognition ||
                          window.msSpeechRecognition;

  var recognition = new SpeechRecognition();
}
catch(e) {
  console.error(e);
  $('.no-browser-support').show();
  $('.app').hide();
}


var interimTextArea = $('#interim-textarea');
var interimContent = '';
var finalTextArea = $('#final-textarea');
var finalContent = '';
var instructions = $('#recording-instructions');
var notesList = $('ul#notes');
var final_transcript = '';
var currTranscript = '';
var w1;
var w2;
var w3;

var times=0;
var camNum=0;
var buttonNum=0;

recognition.lang = 'en-US';
recognition.continuous = true;
recognition.interimResults = true;
recognition.maxAlternatives = 1;

// This block is called every time the Speech APi captures a line. 
recognition.onresult = function(event) {
    var interim_transcript = '';
    for (var i = event.resultIndex; i < event.results.length; ++i) {
      if (event.results[i].isFinal) {
        currTranscript = event.results[i][0].transcript;
        final_transcript += times+'. '+currTranscript+'\n';
      } else {
        interim_transcript += event.results[i][0].transcript;
      }
    }
    var len=0;
    len=currTranscript.split(" ").length;

    // two word command, 1st time
    if(times==0 && len==2){
        w1 =currTranscript.split(" ")[0];
        w2=currTranscript.split(" ")[1];
    // two word command, 2nd time
    }else if(times>0 && len==3){
        w1 =currTranscript.split(" ")[1];
        w2=currTranscript.split(" ")[2];
    }// three word command, 1st time
    else if(times==0 && len==3){
        w1 =currTranscript.split(" ")[0];
        w2=currTranscript.split(" ")[1];
        w3=currTranscript.split(" ")[2];
    }// three word command, 2nd time
    else if(times>0 && len==4){
        w1 =currTranscript.split(" ")[1];
        w2=currTranscript.split(" ")[2];
        w3=currTranscript.split(" ")[3];
    }else{
        w1 ="undefined";
        w2= "undefined";
        w3= "undefined";
    }

    // camera names
    switch(w1) {
        case "top":
            camNum=1;
            break;
        case "side":
            camNum=2;
            break;
        case "gripper":
            camNum=3;
            break;
        default:
            camNum=4;
            break;
     }

    // buttons names
    switch(w2) {
        case "up":
            buttonNum=1;
            break;
        case "right":
            buttonNum=2;
            break;
        case "down":
            buttonNum=3;
            break;
        case "left":
            buttonNum=4;
            break;
        case "rotate":
            if (w2=="left"){
                buttonNum=5;
            }else{
                buttonNum=6;
            }
            break;
        default:
            buttonNum=7;
            break;
     }

    var btnCom=currTranscript;
    var number=0;

    switch(btnCom) {
        case "up":
            number=1;
            break;
        case "down":
            number=2;
            break;
        case "right":
            number=3;
            break;
        case "left":
            number=4;
            break;
     }

    finalContent = 'camNum = '+ camNum +'\nbuttonNum = '+ buttonNum
     +'\nw1 = '+ w1 + '\nw2 ='+ w2 + '\nw3 ='+ w3;// currTranscript + '. len ='+ len;

    interimContent= interim_transcript;
    finalTextArea.val(finalContent);
    interimTextArea.val(interimContent);
    currTranscript = '';
    times++;

var up1BTN = document.getElementById("up");
var up2BTN = document.getElementById("up2");
var right1BTN = document.getElementById("right");
var right2BTN = document.getElementById("right2");
var down1BTN = document.getElementById("down");
var down2BTN = document.getElementById("down2");
var left1BTN = document.getElementById("left");
var left2BTN = document.getElementById("left2");
var rotateLeft1BTN = document.getElementById("rotateLeft");
var rotateLeft2BTN = document.getElementById("rotateLeft2");
var rotateRight1BTN = document.getElementById("rotateRight");
var rotateRight2BTN = document.getElementById("rotateRight2");

    if(camNum==1 && buttonNum ==1){
        up1BTN.click();
    }else if(camNum==1 && buttonNum ==2){
        right1BTN.click();
    }else if(camNum==1 && buttonNum ==3){
        down1BTN.click();
    }else if(camNum==1 && buttonNum ==4){
        left1BTN.click();
    }else if(camNum==1 && buttonNum ==5){
        rotateLeft1BTN.click();
    }else if(camNum==1 && buttonNum ==6){
        rotateRight1BTN.click();
    }else if(camNum==2 && buttonNum ==1){
        up2BTN.click();
    }else if(camNum==2 && buttonNum ==2){
        right2BTN.click();
    }else if(camNum==2 && buttonNum ==3){
        down2BTN.click();
    }else if(camNum==2 && buttonNum ==4){
        left2BTN.click();
    }else if(camNum==2 && buttonNum ==5){
        rotateLeft2BTN.click();
    }else if(camNum==2 && buttonNum ==6){
        rotateRight2BTN.click();
    }


    w1 ="undefined";
    w2= "undefined";
    w3= "undefined";
};

recognition.onstart = function() { 
  instructions.text('Voice recognition is on.');
}

recognition.onspeechend = function() {
  instructions.text('Voice recognition is off.');
}

recognition.onerror = function(event) {
  if(event.error == 'no-speech') {
    instructions.text('No speech was detected. Try again.');  
  };
}



/*-----------------------------
      App buttons and input
------------------------------*/

$('#start-record-btn').on('click', function(e) {


   var element = document.getElementById("start-record-btn");
   $(element).toggleClass('stopStyle');

        if ($(element).hasClass("stopStyle")) {
            element.innerHTML="Stop Recognition";
            recognition.start();
        } else {
            element.innerHTML="Start Recognition";
            recognition.stop();
            instructions.text('Voice recognition paused.');
        }
});



// Sync the text inside the text area with the interimContent variable.
interimTextArea.on('input', function() {
  interimContent = $(this).val();
})

// Sync the text inside the text area with the finalContent variable.
finalTextArea.on('input', function() {
  finalContent = $(this).val();
})


    });