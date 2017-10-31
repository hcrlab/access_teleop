$(document).ready(function(){



Snap.plugin(function (Snap, Element, Paper, global, Fragment) {
    function dragStart(x, y, e) {
        this.current_transform = this.transform();
    }

    function dragMove(dx, dy, x, y, e) {
        this.transform(this.current_transform+'T'+dx+','+dy);
        this.updatePaths();
    }

    function dragEnd(e) {
        this.current_transform = this.transform();
    }

    function updatePaths() {
        var key;
        for(key in this.paths) {
            this.paths[key][0].attr({"path" : this.getPathString(this.paths[key][1])});
            this.paths[key][0].prependTo(this.paper);
        }
    }

    function getCoordinates() {
        return [this.matrix.e + (this.node.width.baseVal.value / 2),
          this.matrix.f + (this.node.height.baseVal.value / 2)];
    }

    function getPathString(obj) {
        var p1 = this.getCoordinates();
        var p2 = obj.getCoordinates();
        return "M"+p1[0]+","+p1[1]+"L"+p2[0]+","+p2[1];
    }

    function addPath(obj) {
        var id = obj.id;
        var path = this.paper.path(this.getPathString(obj)).attr({fill:'none', stroke:'blue', strokeWidth:1});
        path.prependTo(this.paper);
        this.paths[id] = [path, obj];
        obj.paths[this.id] = [path, this];
    }

    function removePath(obj) {
    		var id = obj.id;
        if (this.paths[id] != null) {
        		this.paths[id][0].remove();
            this.paths[id][1] = null;
            delete this.paths[id];

            obj.paths[this.id][1] = null;
            delete obj.paths[this.id];
        }
    }

    Paper.prototype.draggableRect = function (x, y, w, h) {
        var rect = this.rect(0,0,w,h).transform("T"+x+","+y);
        rect.paths = {};
        rect.drag(dragMove, dragStart, dragEnd);
        rect.updatePaths = updatePaths;
        rect.getCoordinates = getCoordinates;
        rect.getPathString = getPathString;
        rect.addPath = addPath;
        rect.removePath = removePath;

        return rect;
    };
});


    function addPath(obj1, obj2) {
        var id = obj1.id;
        //alert(id);
    //    var path = papers.path(this.getPathString(obj)).attr({fill:'none', stroke:'blue', strokeWidth:1});
      //  path.prependTo(this.paper);
   //     this.paths[id] = [path, obj];
 //    .   obj.paths[this.id] = [path, this];
    }
var papers = Snap("#svgCam1");
var circle1 = papers.circle(80, 80, 10).attr({ fill: "red" });
var circle2 = papers.circle(75, 50, 20).attr({ fill: "blue" });
//var rect1 = papers.rect(0,0,40,40);
//rect1.id="rect1";
//var rect2 = papers.draggableRect(100,100,40,40);
//var rect3 = papers.draggableRect(100,0,40,40);
var paper = Snap("#svgCam1");

var x,y, newX, newY, startX=0,startY=0, endX=0, endY=0,c;

function getCoordinates(obj) {
        return [obj.matrix.e + (obj.node.width.baseVal.value / 2),
          obj.matrix.f + (obj.node.height.baseVal.value / 2)];
    }

var arrow = paper.polygon([0,10, 4,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r90');
var marker = arrow.marker(0,0, 10,10, 0,5);

var line = paper.line(0,0,100,100)
    .attr({strokeWidth:5,stroke:"black",strokeLinecap:"round",markerEnd: marker});

var circle = paper.circle(100,100,10).attr({fill:"green"});
var strPath="M"+startX+","+startY+"L"+endX+","+endY;
var triPath="M"+startX+","+startY+"L"+endX+","+endY;
var tri = paper.path("M 0 0 L 10 5 L 0 10 z");
tri.attr({
    id:"tri",
    fill:"#555555"
});
//"M8,247 Q353,242 488,3"

var arrow = paper.polygon([70,10, 84,10, 2,0, 0,10]).attr({fill: '#FFF'}).transform('r270');
var marker = arrow.marker(0,0, 10,10, 0,5);
var path = paper.path(strPath).attr({ fill: "none", stroke: "red", strokeWidth:5,opacity: "1" });

    function dragStart() {
        x = line.attr("x2");
        y = line.attr("y2");
    }

    function dragMove(dx, dy) {
        newX=+x+dx;
        newY=+y+dy;
        circle.attr({cx:newX,cy:newY});
        line.attr({x2:newX,y2:newY});
    }

    function dragEnd() {
        line.animate({x1:newX,y1:newY}, 7000);
    }

    circle.drag(dragMove, dragStart, dragEnd);
  //  paper.mousemove()

function moveFunc( ev, x, y ) {
        newX=+x+dx;
        newY=+y+dy;
        circle.attr({cx:newX,cy:newY});
        line.attr({x2:newX,y2:newY});
};

//paper.mousemove( moveFunc );
});