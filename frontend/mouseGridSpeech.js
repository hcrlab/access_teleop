
$(document).ready(function(){
try {
    var SpeechRecognition = window.SpeechRecognition ||
                          window.webkitSpeechRecognition ||
                          window.mozSpeechRecognition ||
                          window.oSpeechRecognition ||
                          window.msSpeechRecognition;

    var SpeechGrammarList = SpeechGrammarList || webkitSpeechGrammarList;
  var recognition = new SpeechRecognition();
} catch(e) {
  console.error(e);
  $('.no-browser-support').show();
  $('.app').hide();
}


    var numbers = [ 'one' , 'two' , 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine'];
    var grammar = '#JSGF V1.0; grammar numbers; public <number> = ' + numbers.join(' | ') + ' ;'
var interimContent = '';
var finalTextArea = $('#final-textarea');
var finalContent = '';
    var speechRecognitionList = new SpeechGrammarList();
    speechRecognitionList.addFromString(grammar, 1);
    recognition.grammars = speechRecognitionList;
recognition.lang = 'en-US';
recognition.continuous = true;
recognition.interimResults = false;
recognition.maxAlternatives = 1;
    var w1;
    var w2;
    var w3;

    var times=0;
    var cam1LvlCnt=0;
    var cam2LvlCnt=0;
    var camNum=0;
    var buttonNum=0;

    var oneBTN1 = document.getElementById("oneBTN1");
    var twoBTN1 = document.getElementById("twoBTN1");
    var threeBTN1 = document.getElementById("threeBTN1");
    var fourBTN1 = document.getElementById("fourBTN1");
    var fiveBTN1 = document.getElementById("fiveBTN1");
    var sixBTN1 = document.getElementById("sixBTN1");
    var sevBTN1 = document.getElementById("sevBTN1");
    var eigBTN1 = document.getElementById("eigBTN1");
    var nineBTN1 = document.getElementById("nineBTN1");
    var backBTN1 = document.getElementById("backBTN1");
    var forwardBTN1 = document.getElementById("forwardBTN1");

    var oneBTN2 = document.getElementById("oneBTN2");
    var twoBTN2 = document.getElementById("twoBTN2");
    var threeBTN2 = document.getElementById("threeBTN2");
    var fourBTN2 = document.getElementById("fourBTN2");
    var fiveBTN2 = document.getElementById("fiveBTN2");
    var sixBTN2 = document.getElementById("sixBTN2");
    var sevBTN2 = document.getElementById("sevBTN2");
    var eigBTN2 = document.getElementById("eigBTN2");
    var nineBTN2= document.getElementById("nineBTN2");
    var backBTN2 = document.getElementById("backBTN2");
    var forwardBTN2 = document.getElementById("forwardBTN2");

    var currTranscript = '';
// This block is called every time the Speech APi captures a line. 
recognition.onresult = function(event) {

    var last = event.results.length - 1;
    var number = event.results[last][0].transcript;
    for (var i = event.resultIndex; i < event.results.length; ++i) {
        if (event.results[i].isFinal) {
            currTranscript = event.results[i][0].transcript;
            times++;
            //     final_transcript += times+'. '+currTranscript+'\n';
        } else {
            //       interim_transcript += event.results[i][0].transcript;
        }
    }


    //currTranscript = " yes";
    //currTranscript = event.results[i][0].transcript;
    var len = 0;
    len = currTranscript.split(" ").length;
    if (currTranscript.startsWith(" ") && len==2) {
        w2 = currTranscript.split(" ")[1];
   //  alert(currTranscript+" len="+ len);
    }else if (currTranscript.startsWith(" ") && len==3) {
        w1 = currTranscript.split(" ")[1];
        w2 = currTranscript.split(" ")[2];
//        alert(currTranscript+" len="+ len);
    }else if (!currTranscript.startsWith(" ") && len==1) {
        w2 = currTranscript.split(" ")[0];
//        alert(currTranscript+" len="+ len);
    }else if (!currTranscript.startsWith(" ") && len==2) {
        w1 = currTranscript.split(" ")[0];
        w2 = currTranscript.split(" ")[1];
    }else {
        w1 = "undefined";
        w2 = "undefined";
    }
    /*if (len == 2 && times == 1) {
        w1 = currTranscript.split(" ")[0];
        w2 = currTranscript.split(" ")[1];
    } else if (len == 3 && times == 1) {
        w1 = currTranscript.split(" ")[1];
        w2 = currTranscript.split(" ")[2];
    }  else if (len == 2 && times > 1) {
        w2 = currTranscript.split(" ")[1];
    } else {
        w1 = "undefined";
        w2 = "undefined";
    }*/
 //   currTranscript + " t=" + times + "  l=" + len + " w1=" + w1 + " w2=" + w2
    finalTextArea.val(currTranscript);

        // camera names
        switch (w1) {
            case "top":
                camNum = 1;
                break;
            case "side":
                camNum = 2;
                break;
            case "size":
                camNum = 2;
                break;
            case "gripper":
                camNum = 3;
                break;

            default:
                camNum = 6;
                break;
        }

    // buttons names
    switch (w2) {
        case "back":
            buttonNum = 11;
            break;
        case "forward":
            buttonNum = 12;
            break;
        case "one":
            buttonNum = 1;
            break;
        case "two":
            buttonNum = 2;
            break;
        case "to":
            buttonNum = 2;
            break;
        case "three":
            buttonNum = 3;
            break;
        case "four":
            buttonNum = 4;
            break;
        case "for":
            buttonNum = 4;
            break;
        case "five":
            buttonNum = 5;
            break;
        case "six":
            buttonNum = 6;
            break;
        case "seven":
            buttonNum = 7;
            break;
        case "eight":
            buttonNum = 8;
            break;
        case "hate":
            buttonNum = 8;
            break;
        case "ate":
            buttonNum = 8;
            break;
        case "nine":
            buttonNum = 9;
            break;
        default:
            buttonNum = 10;
            break;
    }

    if (!isNaN(w2)) {
        buttonNum = w2;
    }

    if (camNum == 1 && buttonNum == 1) {
        _.debounce(oneBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 2) {
        _.debounce(twoBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 3) {
        _.debounce(threeBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 4) {
        _.debounce(fourBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 5){
        _.debounce(fiveBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    }else if (camNum == 1 && buttonNum == 6) {
        _.debounce(sixBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 7) {
        _.debounce(sevBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 8) {
        _.debounce(eigBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 9) {
        _.debounce(nineBTN1.click(), 1000, true);
        GlobalVal.incrCam1Lev();
    } else if (camNum == 1 && buttonNum == 11) {
        _.debounce(backBTN1.click(), 1000, true);
 //       alert("back 1");
        GlobalVal.decrCam1Lev();
    } else if (camNum == 1 && buttonNum == 12) {
        _.debounce(forwardBTN1.click(), 1000, true);
       // alert("forward 1");
        GlobalVal.decrCam1Lev();
    } else if (camNum == 2 && buttonNum == 1) {
        _.debounce(oneBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 2) {
        _.debounce(twoBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 3) {
        _.debounce(threeBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 4) {
        _.debounce(fourBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 5) {
        _.debounce(fiveBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 6) {
        _.debounce(sixBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 7) {
        _.debounce(sevBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 8) {
        _.debounce(eigBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 9) {
        _.debounce(nineBTN2.click(), 1000, true);
        GlobalVal.incrCam2Lev();
    } else if (camNum == 2 && buttonNum == 11) {
        _.debounce(backBTN2.click(), 1000, true);
   //     alert("back 2");
        GlobalVal.decrCam2Lev();
    } else if (camNum == 2 && buttonNum == 12) {
        _.debounce(forwardBTN2.click(), 1000, true);
  //      alert("forward 2");
        GlobalVal.incrCam2Lev();
    }

  //  w1 = "";
  //  w2 = "";

    if(cam1LvlCnt==3){
        cam1LvlCnt=0;
        times=0;
    }

    if(cam2LvlCnt==3){
        cam2LvlCnt=0;
        times=0;
    }
    // finalTextArea.val('Text: ' + number  + '. camNum='+camNum
    //     + '. buttonNum='+buttonNum);

 //   console.log('Confidence: ' + event.results[0][0].confidence);
};

recognition.onstart = function() {
    finalTextArea.val('Voice recognition is on.');
}

recognition.onspeechend = function() {
    finalTextArea.val('Voice recognition is off.');
    times =0;
}

recognition.onerror = function(event) {
  if(event.error == 'no-speech') {
      finalTextArea.val('No speech was detected. Try again.');
  };
}



/*-----------------------------
      App buttons and input
------------------------------*/



$('#start-record-btn').on('click', function(e) {
   var element = document.getElementById("start-record-btn");
   var state = element.classList.toggle('startStyle');

    if (state) {
        element.innerHTML="Start Recognition";
        recognition.stop();
    } else {
        element.innerHTML="Stop Recognition";
            recognition.start();
        }
});
/**/


// Sync the text inside the text area with the finalContent variable.
finalTextArea.on('input', function() {
  finalContent = $(this).val();
})


    });