
var GlobalVal={
    camera1level:0,
    camera2level:0,

   resetCam1Lev:function() {
       this.camera1level = 0;
    },

    resetCam2Lev:function() {
        this.camera2level = 0;
    },

    incrCam1Lev: function() {
        this.camera1level++;
       // alert("increased");
    },

    incrCam2Lev:function() {
        this.camera2level++;
    },

    decrCam1Lev:function() {
        this.camera1level--;
    },

    decrCam2Lev:function() {
        this.camera2level--;
    },

    get getCam1Lev() {
        return this.camera1level;
    },

    get getCam2Lev() {
        return this.camera2level;
    },

    set setCam1Lev(num) {
        this.camera1level = num;
    },

    set setCam2Lev(num) {
        this.camera2level = num;
    }
};
