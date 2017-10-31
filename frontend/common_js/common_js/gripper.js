/**
 * Created by timadamson on 8/29/17.
 */


Gripper = function (ros) {

    var self = this;

    this.MIN_EFFORT = 35;
    this.MAX_EFFORT = 100;

    this.CLOSED_POS = 0.0;
    this.OPEN_POS = 0.10;

    var currentPosNum;
    var currentPosition;

    this.PositionEnum = {
        OPEN : 0,
        CLOSED : 1,
        PARTLY_CLOSED : 2
    };

    // Set up the gripper publisher
    var gripper_controller = new ROSLIB.ActionClient({
       ros: ros,
       serverName: 'gripper_controller/gripper_action', // If robot has 2 arms, make sure to choose the appropriate one
       actionName : 'control_msgs/GripperCommandAction'
    });

    // A function for sending a goal to the gripper_action
    this.moveGripper = function (position, max_effort) {
        position = (position > self.OPEN_POS)? self.OPEN_POS : position;
        position = (position < self.CLOSED_POS)? self.CLOSED_POS : position;
        var goal = new ROSLIB.Goal({
            actionClient: gripper_controller,
            goalMessage: {
                command: {
                    position : position,
                    max_effort : max_effort
                }
            }
        });
        goal.on('feedback', function(feedback){
            //console.log('Feedback: ' + feedback.position);
            if(self.gripperGUI) {
                self.gripperGUI.adjustGUI(feedback.position);
            }
            currentPosNum = feedback.position;
        });
        goal.on('result', function(result){
            //console.log('Result reached goal: ' + result.reached_goal);
            if(position == self.OPEN_POS && result.reached_goal){
                currentPosition = self.PositionEnum.OPEN;
            }
            else if(position == self.CLOSED_POS && result.reached_goal){
                currentPosition = self.PositionEnum.CLOSED;
            }
            else{
                currentPosition = self.PositionEnum.PARTLY_CLOSED;
            }
            //console.log("The current position is : " + currentPosition);
        });
        goal.send();
    };

    this.open = function () {
        this.moveGripper(this.OPEN_POS, this.MAX_EFFORT);
    };

    this.close = function (max_effort) {
        if(max_effort == undefined || max_effort > 100){
            max_effort = 100;
        }
        this.moveGripper(this.CLOSED_POS, max_effort);
    };

    this.getCurrentPosition = function(){
        return currentPosition;
    };

    this.getCurrentPosNum = function(){
        return currentPosNum;
    };

    this.open(); // A grippers created will start open

};