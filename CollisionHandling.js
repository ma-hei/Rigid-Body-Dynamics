var tempEdge = new Array(3);
var tempEdge2 = new Array(3);
var tempEdge3 = new Array(3);
var tempEdge4 = new Array(3);
var tempPoint = new Array(3);
var tempPoint2 = new Array(3);
var tempCounter=0;
var sqrt2 = Math.sqrt(2);
var tempSwapValue;
var tempSwapIndex;
var xAxisOverlap = new Array(numberBodies);
var yAxisOverlap = new Array(numberBodies);
for (var i=0;i<numberBodies;i++){
    xAxisOverlap[i] = new Array(numberBodies-(i+1));
    yAxisOverlap[i] = new Array(numberBodies-(i+1));
    for (var k=0;k<numberBodies-(i+1);k++){
        xAxisOverlap[i][k]=-1;
        yAxisOverlap[i][k]=-1;
    }
}

var xAxisIntervalsIndex = new Array(numberBodies*2);
var xAxisIntervalsValues = new Array(numberBodies*2);
var yAxisIntervalsIndex = new Array(numberBodies*2);
var yAxisIntervalsValues = new Array(numberBodies*2);

var tempActiveSetx = new Array(numberBodies);
var tempActiveSety = new Array(numberBodies);
for (var i=0;i<numberBodies;i++){
    tempActiveSetx[i] = -1;
    tempActiveSety[i] = -1;
}
var separationInformation = new Array(numberBodies);
for (var i=0;i<numberBodies;i++){
    separationInformation[i] = new Array(numberBodies-(i+1));
    for (var k=0;k<numberBodies-(i+1);k++){
        separationInformation[i][k] = new InformationOfSeparation();
    }
}

var tempIndex1;
var tempIndex2;
var tempBodya;
var tempBodyb;
var crossproductResult = new Array(3);
var crossproductResult2 = new Array(3);
var tempNormalOfEdge = new Array(3);
var positionOfInterpenetrationOnVertex = new Array(3);
var vorzeichen1 = 0;
var vorzeichen2 = 0;
var validCounter = 0;
var tempSeparatingEdge = 0;
var tempedgeStillValid = false;
var newEdgeFound = false;
var tempInterpenetratingVertex = -1;
var tempArray = new Array(3);
var tempArray2 = new Array(3);
var padot = new Array(3);
var pbdot = new Array(3);
var ra = new Array(3);
var rb = new Array(3);
var force = new Array(3);
var vrel;
var numerator;
var epsilon = 0.9;
var term1;
var term2;
var term3;
var term4;
var j;
var collisionCount = 0;
var floorCollisionCount = 0;
var tempSeparatingEdge2 = 0;
var tempSeparatingEdgeStillValid= false;
var tempNewEdgeFound = false;
var alreadydone=false;
var THRESHOLD = 0.0005;
var restingContactCounter=0;

function vectorminus(a,b,r){
    
    r[0] = a[0]-b[0];
    r[1] = a[1]-b[1];
    r[2] = a[2]-b[2];
    
}

function vectorplus(a,b,r){
    
    r[0] = a[0]+b[0];
    r[1] = a[1]+b[1];
    r[2] = a[2]+b[2];
    
}

function vectormul(a,s,r){
    
    r[0] = a[0]*s;
    r[1] = a[1]*s;
    r[2] = a[2]*s;
}

function crossproduct(vec1, vec2, vec3){
    
    vec3[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
    vec3[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
    vec3[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];
    
}

function dotproduct(a,b){
    
    return a[0]*b[0]+a[1]*b[1];
}

function initSeparationInformation(){
    for (var i=0;i<numberBodies;i++){
        for (var k=i+1;k<numberBodies;k++){
            
            fillSeparationInformationBetweenBodies(i,k);
           // compute_information_of_separation_between_bodies(i,k);
        }
    }
}

function initIntervalsForBoundingBoxCheck(){
    
    for (var i=0;i<numberBodies;i++){
        
        xAxisIntervalsIndex[i*2+0] = i*2;
        xAxisIntervalsIndex[i*2+1] = i*2+1;
        yAxisIntervalsIndex[i*2+0] = i*2;
        yAxisIntervalsIndex[i*2+1] = i*2+1;
        
        xAxisIntervalsValues[i*2+0] = bodies[i].x[0]-sqrt2;
        xAxisIntervalsValues[i*2+1] = bodies[i].x[0]+sqrt2;
        yAxisIntervalsValues[i*2+0] = bodies[i].x[1]-sqrt2;
        yAxisIntervalsValues[i*2+1] = bodies[i].x[1]+sqrt2;
        
    }
    
    insertionSortBoundingBoxes(1);
    findInitialAxisOverlap();
    
    
    //text = window.document.getElementById("textField");
    //text.innerHTML = yAxisIntervalsValues+"</br>"+xAxisIntervalsValues;
    
}

function findInitialAxisOverlap(){
    
    //"xAxisIntervalsIndex[i])/2" is the index in bodies[] of the corresponding rigid body
    //this algorithm is an active set method to find overlaps of the rigid bodies on the x and y axis
    
    
    for (var i=0; i<numberBodies*2;i++){
        if (xAxisIntervalsIndex[i]%2==0){
            tempActiveSetx[(xAxisIntervalsIndex[i])/2]=1;
            for (var k=0;k<numberBodies; k++){
                if (tempActiveSetx[k]==1 && k!=(xAxisIntervalsIndex[i])/2){
                    
                    if (((xAxisIntervalsIndex[i])/2)>k){
                        xAxisOverlap[k][((xAxisIntervalsIndex[i])/2)-(k+1)]=1;
                    }
                    else{
                        xAxisOverlap[((xAxisIntervalsIndex[i])/2)][k-(((xAxisIntervalsIndex[i])/2)+1)]=1;
                    }
                    
                }
            }
        }
        else{
            tempActiveSetx[(xAxisIntervalsIndex[i]-1)/2]=-1;
        }
        
        if (yAxisIntervalsIndex[i]%2==0){
            tempActiveSety[(yAxisIntervalsIndex[i])/2]=1;
            for (var k=0;k<numberBodies; k++){
                if (tempActiveSety[k]==1 && k!=(yAxisIntervalsIndex[i])/2){
                    
                    if (((yAxisIntervalsIndex[i])/2)>k){
                        yAxisOverlap[k][((yAxisIntervalsIndex[i])/2)-(k+1)]=1;
                    }
                    else{
                        yAxisOverlap[((yAxisIntervalsIndex[i])/2)][k-(((yAxisIntervalsIndex[i])/2)-1)];
                        
                    }
                }
            }
            
        }
        else{
            tempActiveSety[(yAxisIntervalsIndex[i]-1)/2]=-1;
        }
    }
    
    for (var i=0;i<numberBodies;i++){
        for (var k=0;k<(numberBodies-(i+1));k++){
            
            separationInformation[i][k].bodya = i;
            separationInformation[i][k].bodyb = k+1;
            
        }
    }
    
    
}

function setxAxisOverlap(i,k){
    
    if (i%2==0){
        tempBodya = i/2;
        tempBodyb = (k-1)/2;
    }
    else{
        tempBodya = (i-1)/2;
        tempBodyb = k/2;
    }
    
    
    
    if (tempBodya<tempBodyb){
        tempIndex1 = tempBodya;
        tempIndex2 = tempBodyb;
        
    }
    else{
        tempIndex1 = tempBodyb;
        tempIndex2 = tempBodya;
    }
    
    
    if (xAxisOverlap[tempIndex1][tempIndex2-(tempIndex1+1)]==-1){
        xAxisOverlap[tempIndex1][tempIndex2-(tempIndex1+1)]=1;
    }
    else{
        xAxisOverlap[tempIndex1][tempIndex2-(tempIndex1+1)]=-1;
    }
    
}

function setyAxisOverlap(i,k){
    
    if (i%2==0){
        tempBodya = i/2;
        tempBodyb = (k-1)/2;
    }
    else{
        tempBodya = (i-1)/2;
        tempBodyb = k/2;
    }
    
    
    
    if (tempBodya<tempBodyb){
        tempIndex1 = tempBodya;
        tempIndex2 = tempBodyb;
        
    }
    else{
        tempIndex1 = tempBodyb;
        tempIndex2 = tempBodya;
    }
    
    
    if (yAxisOverlap[tempIndex1][tempIndex2-(tempIndex1+1)]==-1){
        yAxisOverlap[tempIndex1][tempIndex2-(tempIndex1+1)]=1;
    }
    else{
        yAxisOverlap[tempIndex1][tempIndex2-(tempIndex1+1)]=-1;
    }
    
}

function insertionSortBoundingBoxes(init){
    //text = window.document.getElementById("textField");
    //text.innerHTML = "nein";
    for (var i=1; i<numberBodies*2;i++){
        for (var k=i; (k>0 && xAxisIntervalsValues[k]<xAxisIntervalsValues[k-1]); k--){
            tempSwapValue = xAxisIntervalsValues[k-1];
            xAxisIntervalsValues[k-1] = xAxisIntervalsValues[k];
            xAxisIntervalsValues[k] = tempSwapValue;
            
            tempSwapIndex = xAxisIntervalsIndex[k-1];
            xAxisIntervalsIndex[k-1] = xAxisIntervalsIndex[k];
            xAxisIntervalsIndex[k] = tempSwapIndex;
            
            if (init==0){
                if ((xAxisIntervalsIndex[k-1]+xAxisIntervalsIndex[k])%2==1){
                    setxAxisOverlap(xAxisIntervalsIndex[k-1], xAxisIntervalsIndex[k]);
                }
            }
            
        }
    }
    
    
    for (var i=1; i<numberBodies*2;i++){
        for (var k=i; (k>0 && yAxisIntervalsValues[k]<yAxisIntervalsValues[k-1]); k--){
            tempSwapValue = yAxisIntervalsValues[k-1];
            yAxisIntervalsValues[k-1] = yAxisIntervalsValues[k];
            yAxisIntervalsValues[k] = tempSwapValue;
            
            tempSwapIndex = yAxisIntervalsIndex[k-1];
            yAxisIntervalsIndex[k-1] = yAxisIntervalsIndex[k];
            yAxisIntervalsIndex[k] = tempSwapIndex;
            
            if (init==0){
                if ((yAxisIntervalsIndex[k-1]+yAxisIntervalsIndex[k])%2==1){
                    setyAxisOverlap(yAxisIntervalsIndex[k-1], yAxisIntervalsIndex[k]);
                }
            }
            
        }
    }
}

function updateAxisIntervalsOfRigidBodies(){
    
    for (var i=0; i<numberBodies*2;i++){
        
        if (xAxisIntervalsIndex[i]%2==0){
            xAxisIntervalsValues[i] = bodies[xAxisIntervalsIndex[i]/2].x[0]-sqrt2;
        }
        else{
            xAxisIntervalsValues[i] = bodies[(xAxisIntervalsIndex[i]-1)/2].x[0]+sqrt2;
        }
        
        if (yAxisIntervalsIndex[i]%2==0){
            yAxisIntervalsValues[i] = bodies[(yAxisIntervalsIndex[i]/2)].x[1]-sqrt2;
        }
        else{
            yAxisIntervalsValues[i] = bodies[((yAxisIntervalsIndex[i]-1)/2)].x[1]+sqrt2;
        }
        
    }
    
    insertionSortBoundingBoxes(0);
    
    
}

function checkifedgevalid(a, i, b){
    
    tempPoint = bodies[a].vertexes[i];
    
    if (i==3){
        tempEdge[0] = bodies[a].vertexes[0][0]-bodies[a].vertexes[i][0];
        tempEdge[1] = bodies[a].vertexes[0][1]-bodies[a].vertexes[i][1];
        tempEdge[2] = 0;
    }
    else{
        tempEdge[0] = bodies[a].vertexes[i+1][0]-bodies[a].vertexes[i][0];
        tempEdge[1] = bodies[a].vertexes[i+1][1]-bodies[a].vertexes[i][1];
        tempEdge[2] = 0;
    }
    
    
    if (i==0){
        tempEdge2[0] = bodies[a].vertexes[2][0]-tempPoint[0];
        tempEdge2[1] = bodies[a].vertexes[2][1]-tempPoint[1];
        tempEdge2[2] = 0;
    }
    else if (i==1){
        tempEdge2[0] = bodies[a].vertexes[3][0]-tempPoint[0];
        tempEdge2[1] = bodies[a].vertexes[3][1]-tempPoint[1];
        tempEdge2[2] = 0;
    }
    else if (i==2){
        tempEdge2[0] = bodies[a].vertexes[0][0]-tempPoint[0];
        tempEdge2[1] = bodies[a].vertexes[0][1]-tempPoint[1];
        tempEdge2[2] = 0;
        
    }
    else if (i==3){
        tempEdge2[0] = bodies[a].vertexes[1][0]-tempPoint[0];
        tempEdge2[1] = bodies[a].vertexes[1][1]-tempPoint[1];
        tempEdge2[2] = 0;
    }
    
    crossproduct(tempEdge, tempEdge2, crossproductResult);
    
    if (crossproductResult[2]>0){
        vorzeichen1 = 1;
    }
    else{
        vorzeichen1 = -1;
    }
    
    validCounter = 0;
    
    
    for (var k=0;k<4;k++){
        
        tempEdge3[0] = bodies[b].vertexes[k][0] - tempPoint[0];
        tempEdge3[1] = bodies[b].vertexes[k][1] - tempPoint[1];
        tempEdge3[2] = 0;
        
        crossproduct(tempEdge, tempEdge3, crossproductResult);
        
        
        if (crossproductResult[2]>0){
            vorzeichen2 = 1;
        }
        else{
            vorzeichen2 = -1;
        }
        
        
        if (vorzeichen1!=vorzeichen2){
            
            validCounter++;
            
        }
        
    }
    if (validCounter==4){
        return true;
    }
    
    return false;
}

function find_separating_edge_between_bodies(a,b){
    
    for (var i=0;i<4;i++){
        
        if (check_if_edge_is_separating_bodies(a, i, b)){
            return i;
        }
        
    }
    
    return -1;
}

function findSeparatingEdgeBetweenBodies(a,b){
    
    for (var i=0;i<4;i++){
        
        if (checkifedgevalid(a, i, b)){
            return i;
        }
        
    }
    
    return -1;
}

function fillSeparationInformationBetweenBodies(i,k){
    
    tempSeparatingEdge2 = findSeparatingEdgeBetweenBodies(i,k);
    
    if (tempSeparatingEdge2!=-1){
        separationInformation[i][k-(i+1)].bodya = i;
        separationInformation[i][k-(i+1)].bodyb = k;
        separationInformation[i][k-(i+1)].bodyWithSeparatingEdge = i;
        separationInformation[i][k-(i+1)].separatingEdge = tempSeparatingEdge2;
        separationInformation[i][k-(i+1)].collision = 0;
        return true;
    }
    else{
        tempSeparatingEdge2 = findSeparatingEdgeBetweenBodies(k, i);
        if (tempSeparatingEdge2!=-1){
            separationInformation[i][k-(i+1)].bodya = k;
            separationInformation[i][k-(i+1)].bodyb = i;
            separationInformation[i][k-(i+1)].bodyWithSeparatingEdge = k;
            separationInformation[i][k-(i+1)].separatingEdge = tempSeparatingEdge2;
            return true;
        }
        else{
            
            separationInformation[i][k-(i+1)].collision = 1;
            
            return false;
        }
        
    }
    
}

var index_of_separating_edge;
function compute_information_of_separation_between_bodies(i,k){
    
    index_of_separating_edge = get_separating_edge_between_bodies(i,k);
    
    if (index_of_separating_edge!=-1){
        
        separationInformation[i][k-(i+1)].bodya = k;
        separationInformation[i][k-(i+1)].bodyb = i;
        separationInformation[i][k-(i+1)].bodyWithSeparatingEdge = i;
        separationInformation[i][k-(i+1)].separatingEdge = index_of_separating_edge;
        separationInformation[i][k-(i+1)].collision = 0;
        return true;
    }
    else{
        index_of_separating_edge = get_separating_edge_between_bodies(k,i);
        if (index_of_separating_edge!=-1){
            separationInformation[i][k-(i+1)].bodya = i;
            separationInformation[i][k-(i+1)].bodyb = k;
            separationInformation[i][k-(i+1)].bodyWithSeparatingEdge = k;
            separationInformation[i][k-(i+1)].separatingEdge = index_of_separating_edge;
            separationInformation[i][k-(i+1)].collision = 0;
        return true;
        }
        else{
            separationInformation[i][k-(i+1)].collision = 1;
            return false;
        }
    }
    
    
}

var separating_edge = new Array(3);
function get_separating_edge_between_bodies(i,k){
    
    var index_of_body_with_separating_edge = separationInformation[i][k-(i+1)].bodyWithSeparatingEdge;
    var index_of_separating_edge = separationInformation[i][k-(i+1)].separatingEdge;
    
    if (index_of_separating_edge==3){
        vectorminus(bodies[index_of_body_with_separating_edge].vertexes[0], bodies[index_of_body_with_separating_edge].vertexes[3], separating_edge);
    }
    else{
        vectorminus(bodies[index_of_body_with_separating_edge].vertexes[index_of_separating_edge+1], bodies[index_of_body_with_separating_edge].vertexes[index_of_separating_edge], separating_edge);
    }
    
}

var own_edge = new Array(3);
function get_edge_towards_own_body(a, e){
    
    if (e<2){
        vectorminus(bodies[a].vertexes[e+2], bodies[a].vertexes[e], own_edge);
    }
    else if(e==2){
        vectorminus(bodies[a].vertexes[0], bodies[a].vertexes[2]);
    }
    else if(e==3){
        vectorminus(bodies[a].vertexes[1], bodies[a].vertexes[3]);
    }
    
    
    
}

var temp_edge = new Array(3);
var temp_edge2 = new Array(3);
function find_vertex_of_body_on_other_side_of_edge(a, e, b){
    
    //separating_edge and own_edge need to be set by previous functions
    
    get_separating_edge_between_bodies(a,b);
    get_edge_towards_own_body(a, e);
    
    
    crossproduct(separating_edge, own_edge, crossproductResult);
    if (crossproductResult[2]>0){
        vorzeichen1 = 1;
    }
    else{
        vorzeichen1 = -1;
    }
    
    for (var y=0;y<4;y++){
        
        
        vectorminus(bodies[a].vertexes[e], bodies[b].vertexes[y], temp_edge);
        
        crossproduct(separating_edge, temp_edge, crossproductResult);
        if (crossproductResult[2]>0){
            vorzeichen2 = 1;
        }
        
        else{
            vorzeichen2 = -1;
        }
        if (vorzeichen1 != vorzeichen2){
            return y;
        }
        
        
    }
    
    return -1;
    
    
}

function detectInterpenetratingVertexOfCollision(i,k){
    
    
    if (separationInformation[i][k-(i+1)].separatingEdge==3){
        tempEdge4[0] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[0][0]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[3][0];
        tempEdge4[1] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[0][1]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[3][1];
        tempEdge4[2] = 0;
    }
    else{
        tempEdge4[0] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge+1][0]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge][0];
        tempEdge4[1] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge+1][1]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge][1];
        tempEdge4[2] = 0;
    }
    
    
    if (separationInformation[i][k-(i+1)].separatingEdge<2){
        tempEdge2[0] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge+2][0] - bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge][0];
        tempEdge2[1] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge+2][1] - bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge][1];
        tempEdge2[2] = 0;
    }
    else if(separationInformation[i][k-(i+1)].separatingEdge==2){
        tempEdge2[0] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[0][0]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[2][0];
        tempEdge2[1] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[0][1]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[2][1];
        tempEdge2[2] = 0;
    }
    else if(separationInformation[i][k-(i+1)].separatingEdge==3){
        tempEdge2[0] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[1][0]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[3][0];
        tempEdge2[1] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[1][1]-bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[3][1];
        tempEdge2[2] = 0;
        
    }
    
    crossproduct(tempEdge4, tempEdge2, crossproductResult);
    if (crossproductResult[2]>0){
        vorzeichen1 = 1;
    }
    else{
        vorzeichen1 = -1;
    }
    
    for (var y=0;y<4;y++){
        
        tempEdge2[0] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge][0] - bodies[separationInformation[i][k-(i+1)].bodyb].vertexes[y][0];
        tempEdge2[1] = bodies[separationInformation[i][k-(i+1)].bodyWithSeparatingEdge].vertexes[separationInformation[i][k-(i+1)].separatingEdge][1] - bodies[separationInformation[i][k-(i+1)].bodyb].vertexes[y][1];
        tempEdge2[2] = 0;
        
        crossproduct(tempEdge4, tempEdge2, crossproductResult);
        if (crossproductResult[2]>0){
            vorzeichen2 = 1;
        }
        
        else{
            vorzeichen2 = -1;
        }
        if (vorzeichen1 != vorzeichen2){
            return y;
        }
        
        
    }
    
    return -1;
    
}

var position_of_interpenetration_of_vertex = new Array(3);
function compute_position_of_interpenetration_of_vertex_on_edge(v,i,k){
    
    //separating_edge needs to be set
    //normal_of_interpenetrated_edge needs to be set
    
    var bodya = separationInformation[i][k-(i+1)].bodya;
    var bodyb = separationInformation[i][k-(i+1)].bodyb;
    var edge = separationInformation[i][k-(i+1)].separatingEdge;
    
    vectorminus(bodies[bodya].vertexes[v], bodies[bodyb].vertexes[edge], temp_edge);
    
    crossproduct(separating_edge, temp_edge, crossproductResult);
    
    var length1 = vectorlength(temp_edge);
    var length2 = vectorlength(crossproductResult);
    
    var distance = length2/length1;
    
    vectormul(normal_of_interpenetrated_edge, distance, temp_edge);
    vectorplus(bodies[a].vertexes[v], temp_edge);
    
    
}

var normal_of_interpenetrated_edge = new Array(3);
function compute_normal_of_interpenetrated_edge(i,k){
    
    //separating_edge needs to be set
    
    var bodyb = separationInformation[i][k-(i+1)].bodyb;
    var edge = separationInformation[i][k-(i+1)].separatingEdge;
    
    if (separating_edge[0]!=0){
        normal_of_interpenetrated_edge[2] = 0;
        normal_of_interpenetrated_edge[1] = 1;
        normal_of_interpenetrated_edge[0] = -(separating_edge[1]/separating_edge[0]);
    }
    else{
        normal_of_interpenetrated_edge[2] = 0;
        normal_of_interpenetrated_edge[1] = -(separating_edge[0]/separating_edge[1]);
        normal_of_interpenetrated_edge[0] = 1;
    }
    
    var length = vectorlength(normal_of_interpenetrated_edge);
    
    normal_of_interpenetrated_edge[1] = normal_of_interpenetrated_edge[1]/length;
    normal_of_interpenetrated_edge[0] = normal_of_interpenetrated_edge[0]/length;
    
    vectorminus(bodies[bodyb].x, bodies[bodyb].vertexes[edge], temp_edge);
    
    crossproduct(separating_edge, temp_edge, crossproductResult);
    
    temp_edge[0] = normal_of_interpenetrated_edge[0];
    temp_edge[1] = normal_of_interpenetrated_edge[1];
    temp_edge[2] = 0;
    
    crossproduct(separating_edge, temp_edge, crossproductResult2);
    
    if ((crossproductResult[2]>0 && crossproductResult2[2]>0) || (crossproductResult[2]<0 && crossproductResult2[2]<0)){
        normal_of_interpenetrated_edge[0]*=-1;
        normal_of_interpenetrated_edge[1]*=-1;
        return;
    }
    else{
        return;
    }
    
}

var temp_point = new Array(3);
function check_if_edge_is_separating_bodies(a, e, b){
    
    if (e==3){
        vectorminus(bodies[a].vertexes[0], bodies[a].vertexes[e], temp_edge);
    }
    else{
        vectorminus(bodies[a].vertexes[e+1], bodies[a].vertexes[e], temp_edge);
    }
    
    if (e==0){
        vectorminus(bodies[a].vertexes[2], bodies[a].vertexes[e], temp_edg2);
    }
    else if (e==1){
        vectorminus(bodies[a].vertexes[3], bodies[a].vertexes[e], temp_edg2);
    }
    else if (e==2){
        vectorminus(bodies[a].vertexes[0], bodies[a].vertexes[e], temp_edg2);
        
    }
    else if (e==3){
        vectorminus(bodies[a].vertexes[1], bodies[a].vertexes[e], temp_edg2);
    }
    
    crossproduct(temp_edge, temp_edge2, crossproductResult);
    
    if (crossproductResult[2]>0){
        vorzeichen1 = 1;
    }
    else{
        vorzeichen1 = -1;
    }
    
    validCounter = 0;
    
    
    for (var k=0;k<4;k++){
        
        vectorminus(bodies[b].vertexes[k], bodies[a].vertexes[e], temp_edge2);
        
        crossproduct(temp_edge, temp_edge2, crossproductResult);
        
        
        if (crossproductResult[2]>0){
            vorzeichen2 = 1;
        }
        else{
            vorzeichen2 = -1;
        }
        
        
        if (vorzeichen1!=vorzeichen2){
            
            validCounter++;
            
        }
        
    }
    if (validCounter==4){
        return true;
    }
    
    return false;
    
}

function computePositionOfInterpenetrationOnVertex(i,k){
    
    //tempEdge4 still has the correct edge vertex from the previously called function detectInterpenetratingVertex()
    
    tempEdge3[0] = bodies[separationInformation[i][k-(i+1)].bodyb].vertexes[tempInterpenetratingVertex][0] - bodies[separationInformation[i][k-(i+1)].bodya].vertexes[separationInformation[i][k-(i+1)].separatingEdge][0];
    tempEdge3[1] = bodies[separationInformation[i][k-(i+1)].bodyb].vertexes[tempInterpenetratingVertex][1] - bodies[separationInformation[i][k-(i+1)].bodya].vertexes[separationInformation[i][k-(i+1)].separatingEdge][1];
    tempEdge3[2] = 0;
    
    crossproduct(tempEdge4, tempEdge3, crossproductResult);
    
    var length = tempEdge4[0]*tempEdge4[0]+tempEdge4[1]*tempEdge4[1]+tempEdge4[2]*tempEdge4[2];
    length = Math.sqrt(length);
    
    var length2 = crossproductResult[0]*crossproductResult[0] + crossproductResult[1]*crossproductResult[1]+ crossproductResult[2]*crossproductResult[2];
    length2 = Math.sqrt(length2);
    
    var distance = length2/length;
    positionOfInterpenetrationOnVertex[0] = bodies[separationInformation[i][k-(i+1)].bodyb].vertexes[tempInterpenetratingVertex][0]+tempNormalOfEdge[0]*distance;
    positionOfInterpenetrationOnVertex[1] = bodies[separationInformation[i][k-(i+1)].bodyb].vertexes[tempInterpenetratingVertex][1]+tempNormalOfEdge[1]*distance;
    positionOfInterpenetrationOnVertex[2] = 0;
    
}

function computeNormalOfPenetratedEdge(i,k){
    
    //tempEdge4 still has the correct edge vertex from the previously called function detectInterpenetratingVertex()
    
    if(tempEdge4[0]!=0){
        tempNormalOfEdge[2] = 0;
        tempNormalOfEdge[1] = 1;
        tempNormalOfEdge[0] = -(tempEdge4[1]/tempEdge4[0]);
    }
    else{
        tempNormalOfEdge[2] = 0;
        tempNormalOfEdge[1] = -(tempEdge4[0]/tempEdge4[1]);
        tempNormalOfEdge[0] = 1;
    }
    
    
    
    
    var length = tempNormalOfEdge[0]*tempNormalOfEdge[0]+tempNormalOfEdge[1]*tempNormalOfEdge[1];
    length = Math.sqrt(length);
    
    
    tempNormalOfEdge[1] = tempNormalOfEdge[1]/length;
    tempNormalOfEdge[0] = tempNormalOfEdge[0]/length;
    
    //tempEdge2 gets the starting point from which the edge goes out
    //tempEdge2 = bodies[overlaps[i][k-(i+1)].bodya].vertexes[overlaps[i][k-(i+1)].separatingEdge];
    tempEdge3[0] = bodies[separationInformation[i][k-(i+1)].bodya].x[0] - bodies[separationInformation[i][k-(i+1)].bodya].vertexes[separationInformation[i][k-(i+1)].separatingEdge][0];
    tempEdge3[1] = bodies[separationInformation[i][k-(i+1)].bodya].x[1] - bodies[separationInformation[i][k-(i+1)].bodya].vertexes[separationInformation[i][k-(i+1)].separatingEdge][1];
    tempEdge3[2] = 0;
    
    crossproduct(tempEdge4, tempEdge3, crossproductResult);
    
    tempEdge3[0] = tempNormalOfEdge[0];
    tempEdge3[1] = tempNormalOfEdge[1];
    tempEdge3[2] = 0;
    
    crossproduct(tempEdge4, tempEdge3, crossproductResult2);
    
    
    
    if ((crossproductResult[2]>0 && crossproductResult2[2]>0) || (crossproductResult[2]<0 && crossproductResult2[2]<0)){
        tempNormalOfEdge[0]*=-1;
        tempNormalOfEdge[1]*=-1;
        return;
    }
    else{
        return;
    }
    
    
    
}

function pt_velocity(i,p, velocityOfPointOnBody){
    
    tempArray[0] = p[0] - bodies[i].x[0];
    tempArray[1] = p[1] - bodies[i].x[1];
    tempArray[2] = 0;
    
    crossproduct(bodies[i].omega, tempArray, tempArray2);
    
    velocityOfPointOnBody[0] = bodies[i].v[0] + tempArray2[0];
    velocityOfPointOnBody[1] = bodies[i].v[1] + tempArray2[1];
    velocityOfPointOnBody[2] = 0;
    
}

function initCollision(i,k){
    
    tempInterpenetratingVertex = detectInterpenetratingVertexOfCollision(i,k);
    computeNormalOfPenetratedEdge(i,k);
    computePositionOfInterpenetrationOnVertex(i,k);
    contact = new Contact();
    contact.a = separationInformation[i][k-(i+1)].bodyb;
    contact.b = separationInformation[i][k-(i+1)].bodya;
    contact.p = positionOfInterpenetrationOnVertex.slice(0);
    contact.n = tempNormalOfEdge.slice(0);
    contact.v = tempInterpenetratingVertex;
    collisions[collisionCount] = contact;
    
}

var temp_interpenetrating_vertex;
function init_collision_between_bodies(i,k){
    
    
    temp_interpenetrating_vertex = find_vertex_of_body_on_other_side_of_edge(i, separationInformation[i][k-(i+1)].separatingEdge, k);
    
    
}

var temp_is_edge_still_valid = false;
var temp_new_edge_found = false;
function check_if_bodies_collided(i,k){
    
    if (separationInformation[i][k-(i+1)].collision==0){
        
        temp_is_edge_still_valid = check_if_edge_is_separating_bodies(separationInformation[i][k-(i+1)].bodyb, separationInformation[i][k-(i+1)].separatingEdge, separationInformation[i][k-(i+1)].bodya);
        
        if (!temp_is_edge_still_valid){
            
            temp_new_edge_found = compute_information_of_separation_between_bodies(i,k);
            
            if (!temp_new_edge_found){
                
                
            }
            
        }
        
    }
    else{
        
        temp_new_edge_found = compute_information_of_separation_between_bodies(i,k);
        
        if (!temp_new_edge_found){
        }
        
    }
    
    
    
}

function checkIfBodiesCollided(i,k){
    
    
    if (separationInformation[i][k-(i+1)].collision==0){
        
        tempSeparatingEdgeStillValid = checkifedgevalid(separationInformation[i][k-(i+1)].bodya, separationInformation[i][k-(i+1)].separatingEdge, separationInformation[i][k-(i+1)].bodyb);
        
        if (!tempSeparatingEdgeStillValid){
            
            tempNewEdgeFound = fillSeparationInformationBetweenBodies(i,k);
            
            if (!tempNewEdgeFound){
                initCollision(i,k);
                collisionCount++;
                
            }
            
        }
    }
    else{
        
        tempNewEdgeFound = fillSeparationInformationBetweenBodies(i,k);
        
        if (!tempNewEdgeFound){
            
            initCollision(i,k);
            collisionCount++;
            
        }
        
    }
    
}


function findBoundingBoxOverlap(){
    
    for (var i=0;i<numberBodies-1;i++){
        for (var k=i+1;k<numberBodies;k++){
            if (xAxisOverlap[i][k-(i+1)]==1){
                if (yAxisOverlap[i][k-(i+1)]==1){
                    
                    checkIfBodiesCollided(i,k);
                    
                }
            }
        }
    }
    
    
}


function updateBoundingBoxes(){
    
    updateAxisIntervalsOfRigidBodies();
    findBoundingBoxOverlap();
    
}


function update_bounding_boxes_and_find_collisions(){
    
    updateAxisIntervalsOfRigidBodies();
    findBoundingBoxOverlap();
    
}

function vectorMults(v,s,r){
    
    r[0] = v[0]*s;
    r[1] = v[1]*s;
    r[2] = v[2]*s;
}

function compute_acc_of_point_on_body(a, p, acc){
    
    
    
    var ra = new Array(3);
    vectorminus(p,bodies[a].x,ra);
    
    var omegaCrossRa = new Array(3);
    crossproduct(bodies[a].omega,ra, omegaCrossRa);
    var omegaCrossOmegaCrossRa = new Array(3);
    crossproduct(bodies[a].omega,omegaCrossRa, omegaCrossOmegaCrossRa);
    var wdota = new Array(3);
    wdota = numeric.dot(bodies[a].Iinv,bodies[a].torque);
    var wdotcrossra = new Array(3);
    crossproduct(wdota, ra, wdotcrossra);
    var term1a = new Array(3);
    vectorplus(bodies[a].force, wdotcrossra, term1a);
    vectorplus(term1a, omegaCrossOmegaCrossRa, acc);
    
    
}

function compute_acc_of_distance_between_point_and_point(c){
    
    var acc_on_a = new Array(3);
    compute_acc_of_point_on_body(c.a, bodies[c.a].vertexes[c.v], acc_on_a);
    
    var acc_on_b = new Array(3);
    compute_acc_of_point_on_body(c.b, c.p, acc_on_b);
    
    var acc_on_a_minus_acc_on_b = new Array(3);
    vectorminus(acc_on_a, acc_on_b, acc_on_a_minus_acc_on_b);
    
    var k1 = dotproduct(c.n, acc_on_a_minus_acc_on_b);
    
    var vel_on_a = new Array(3);
    pt_velocity(c.a, bodies[c.a].vertexes[c.v], vel_on_a);
    
    var vel_on_b = new Array(3);
    pt_velocity(c.b, c.p, vel_on_b);
    
    var vel_a_minus_vel_b = new Array(3);
    vectorminus(vel_on_a, vel_on_b, vel_a_minus_vel_b);
    
    var ndot = new Array(3);
    crossproduct(bodies[c.b].omega, c.n, ndot);
    
    var k2 = dotproduct(ndot, vel_a_minus_vel_b);
    var k2 = k2*2;
    
    answer = k1+k2;
    
    var vrel = dotproduct(c.n, vel_on_a, vel_on_b)
    
    
    text = window.document.getElementById("textField");
    text.innerHTML = vrel;
    
    
    return answer;
    
}

function make_resting_contact_stay(a,b,c, answer){
    
    if (answer<0){
    var temp = answer*-1;
    var force = new Array(3);
    var atorqueAdd = new Array(3);
    var btorqueAdd = new Array(3);
    var vectorToVertexOnA = new Array(3);
    var vectorToPointOnB = new Array(3);
    vectorMults(c.n, temp, force);
    
    vectorminus(bodies[a].vertexes[c.v], bodies[a].x, vectorToVertexOnA);
    vectorminus(c.p, bodies[b].x, vectorToPointOnB);
    crossproduct(vectorToVertexOnA, force, atorqueAdd);
    crossproduct(vectorToPointOnB, force, btorqueAdd);
        
    
    bodies[a].force[0]=bodies[a].force[0]+force[0];
    bodies[a].force[1]=bodies[a].force[1]+force[1];
    bodies[a].force[2]=bodies[a].force[2]+force[2];
    
    bodies[a].torque[0]=bodies[a].torque[0]+atorqueAdd[0];
    bodies[a].torque[1]=bodies[a].torque[1]+atorqueAdd[1];
    bodies[a].torque[2]=bodies[a].torque[2]+atorqueAdd[2];
    
    bodies[b].force[0]=bodies[b].force[0]-force[0];
    bodies[b].force[1]=bodies[b].force[1]-force[1];
    bodies[b].force[2]=bodies[b].force[2]-force[2];
    
    bodies[b].torque[0]=bodies[b].torque[0]-btorqueAdd[0];
    bodies[b].torque[1]=bodies[b].torque[1]-btorqueAdd[1];
    bodies[b].torque[2]=bodies[b].torque[2]-btorqueAdd[2];
    }
}

var restingContacts = new Array(100);
var amat;
var tempVector = new Array(3);
var tempVector2 = new Array(3);
var tempVector3 = new Array(3);
var string = "hallo</br>";
var howoften=2;
var firstcall=true;
var called=false;
var test;
function compute_contact_forces(c, n, t){
    
    
        var ra = new Array(3);
        vectorminus(bodies[c[0].a].vertexes[2],bodies[c[0].a].x,ra);
        var rb = new Array(3);
        vectorminus(c[0].p,bodies[c[0].b].x,rb);
    
        var omegaCrossRa = new Array(3);
        crossproduct(bodies[c[0].a].omega,ra, omegaCrossRa);
        var omegaCrossOmegaCrossRa = new Array(3);
        crossproduct(bodies[c[0].a].omega,omegaCrossRa, omegaCrossOmegaCrossRa);
        var wdota = new Array(3);
        wdota = numeric.dot(bodies[c[0].a].Iinv,bodies[c[0].a].torque);
        var wdotcrossra = new Array(3);
        crossproduct(wdota, ra, wdotcrossra);
        var term1a = new Array(3);
        vectorplus(bodies[c[0].a].force, wdotcrossra, term1a);
        vectorplus(term1a, omegaCrossOmegaCrossRa, term1a);
    
    
        
        var omegaCrossRb = new Array(3);
        crossproduct(bodies[c[0].b].omega,rb, omegaCrossRb);
        var omegaCrossOmegaCrossRb = new Array(3);
        crossproduct(bodies[c[0].b].omega,omegaCrossRb, omegaCrossOmegaCrossRb);
        var wdotb = new Array(3);
        wdotb = numeric.dot(bodies[c[0].b].Iinv,bodies[c[0].b].torque);
        var wdotcrossrb = new Array(3);
        crossproduct(wdotb, rb, wdotcrossrb);
        var term1b = new Array(3);
        vectorplus(bodies[c[0].b].force, wdotcrossrb, term1b);
        vectorplus(term1b, omegaCrossOmegaCrossRb, term1b);
    
        
        
    var bigterm = new Array(3);
    vectorminus(term1a, term1b, bigterm);
    
    var k1 = dotproduct(c[0].n, bigterm);
    
    var padot = new Array(3);
    pt_velocity(c[0].a, c[0].p, padot);
    var pbdot = new Array(3);
    pt_velocity(c[0].b, c[0].p, pbdot);
    var term2 = new Array(3);
    vectorminus(padot, pbdot, term2);
    var term1 = new Array(3);
    crossproduct(bodies[c[0].b].omega, c[0].n, term1);
    var k2 = dotproduct(term1, term2);
    k2 = k2*2;
        
        
        
        
        var k2plusk1 = k2+k1;
        string = string+"</br>before this function: </br>";
        string = string+"a.omega: "+bodies[c[0].a].omega+"</br>";
        string = string+"a.force: "+bodies[c[0].a].force+"</br>";
        string = string+"a.torque: "+bodies[c[0].a].torque+"</br>";
        string = string+"b.force: "+bodies[c[0].b].force+"</br>";
        string = string+"b.torque: "+bodies[c[0].b].torque+"</br>";
        
        
        string = string+"</br>accelaration of distance: "+k2plusk1;
        
        if (k2plusk1<0){
    
        var minusk1 = (k2plusk1)*(-1);
        var resultingforce = minusk1;
        var fmuln = new Array(3);
        vectorMults(c[0].n, resultingforce, fmuln);
            
            string = string+"</br>applying force: "+fmuln+"</br>";
            
    
        vectorplus(bodies[c[0].a].force,fmuln,bodies[c[0].a].force);
        crossproduct(ra, fmuln, tempArray);
        vectorplus(bodies[c[0].a].torque, tempArray, bodies[c[0].a].torque);
            
        vectorminus(bodies[c[0].b].force,fmuln,bodies[c[0].b].force);
        crossproduct(rb, fmuln, tempArray);
        vectorminus(bodies[c[0].b].torque, tempArray, bodies[c[0].b].torque);
            
            
            string = string+"</br>after this function: </br>";
            string = string+"a.force: "+bodies[c[0].a].force+"</br>";
            string = string+"a.torque: "+bodies[c[0].a].torque+"</br>";
            string = string+"b.force: "+bodies[c[0].b].force+"</br>";
            string = string+"b.torque: "+bodies[c[0].b].torque+"</br>";
            
            test = minusk1-resultingforce;
            string = string+"test: "+test;
            
        }
    
    
}


function colliding(c){
    
    
    pt_velocity(c.a,c.p,padot);
    pt_velocity(c.b,c.p,pbdot);
    
    vectorminus(padot, pbdot, tempArray);
    
    vrel = dotproduct(c.n, tempArray);
    
    if (vrel>THRESHOLD){
        return false;
    }
    else if (vrel>-1*THRESHOLD){
        return false;
    }
    else{
        return true;
    }
    
}

function colliding_and_put_into_restings(c){
    
    pt_velocity(c.a,c.p,padot);
    pt_velocity(c.b,c.p,pbdot);
    
    vectorminus(padot, pbdot, tempArray);
    
    vrel = dotproduct(c.n, tempArray);
    
    if (vrel>THRESHOLD){
        return false;
    }
    else if (vrel>-1*THRESHOLD){
        restingContacts[restingContactCounter] = c;
        restingContactCounter++;
        return false;
    }
    else{
        return true;
    }
    
}

function collision(c){
    
    pt_velocity(c.a, c.p, padot);
    pt_velocity(c.b, c.p, pbdot);
    vectorminus(c.p, bodies[c.a].x, ra);
    vectorminus(c.p, bodies[c.b].x, rb);
    vectorminus(padot, pbdot, tempArray);
    vrel = dotproduct(c.n, tempArray);
    numerator = -(1+epsilon)*vrel;
    
    
    term1 = 1/(bodies[c.a].mass);
    term2 = 1/(bodies[c.b].mass);
    
    crossproduct(ra, c.n, tempArray);
    crossproduct(numeric.dot(bodies[c.a].Iinv,tempArray),ra, tempArray);
    term3 = dotproduct(c.n, tempArray);
    
    crossproduct(rb, c.n, tempArray);
    crossproduct(numeric.dot(bodies[c.b].Iinv,tempArray),rb, tempArray);
    term4 = dotproduct(c.n, tempArray);
    
    j = numerator/(term1+term2+term3+term4);
    force[0] = j*c.n[0];
    force[1] = j*c.n[1];
    force[2] = 0;
    
    bodies[c.a].P[0] = bodies[c.a].P[0] + force[0];
    bodies[c.a].P[1] = bodies[c.a].P[1] + force[1];
    
    bodies[c.b].P[0] = bodies[c.b].P[0] - force[0];
    bodies[c.b].P[1] = bodies[c.b].P[1] - force[1];
    
    
    crossproduct(ra, force, tempArray);
    bodies[c.a].L[0] = bodies[c.a].L[0] + tempArray[0];
    bodies[c.a].L[1] = bodies[c.a].L[1] + tempArray[1];
    bodies[c.a].L[2] = bodies[c.a].L[2] + tempArray[2];
    
    crossproduct(rb, force, tempArray);
    bodies[c.b].L[0] = bodies[c.b].L[0] - tempArray[0];
    bodies[c.b].L[1] = bodies[c.b].L[1] - tempArray[1];
    bodies[c.b].L[2] = bodies[c.b].L[2] - tempArray[2];
    
    bodies[c.a].v[0] = bodies[c.a].P[0] / bodies[c.a].mass;
    bodies[c.a].v[1] = bodies[c.a].P[1] / bodies[c.a].mass;
    
    bodies[c.b].v[0] = bodies[c.b].P[0] / bodies[c.b].mass;
    bodies[c.b].v[1] = bodies[c.b].P[1] / bodies[c.b].mass;
    
    bodies[c.a].omega = numeric.dot(bodies[c.a].Iinv, bodies[c.a].L );
    bodies[c.b].omega = numeric.dot(bodies[c.b].Iinv, bodies[c.b].L );
    
}

function sortOutRestingContacts(){
    
    for (var i=0;i<collisionCount;i++){
        colliding_and_put_into_restings(collisions[i]);
    }
    
}

var times_contact_called=0;
function handle_all_collisions(){
    
    var had_collision;
    
    
    do{
        had_collision = false;
        
        for (var i = 0;i<collisionCount;i++){
            
            
            if (colliding(collisions[i])){
                
                collision(collisions[i]);
                
                had_collision = true;
                
                
            }
        }
    }
    while(had_collision == true);
    
    collisionCount=0;
}

var floorEdge = new Array(3);
floorEdge[0] = 1;
floorEdge[1] = 0;
floorEdge[2] = 0;
var floorEdge2 = new Array(3);
floorEdge2[0] = 1;
floorEdge2[1] = -1;
floorEdge2[2] = 0;
var floorResult = new Array(3);
crossproduct(floorEdge, floorEdge2, floorResult);
var floorvorzeichen = 0;
if (floorResult[2]>0){
    floorvorzeichen=1;
}
else{
    floorvorzeichen=-1;
}

var tempInterpenetratingFloorVertex = -1;


function floor_colliding(c){
    
    pt_velocity(c.a, bodies[c.a].vertexes[c.interpenetratingVertex],padot);
    tempEdge[0]=0;
    tempEdge[1]=1;
    tempEdge[2]=0;
    vrel = dotproduct(tempEdge,padot);
    
    
    if (vrel>THRESHOLD){
        return false;
    }
    else if (vrel>-1*THRESHOLD){
        
        return false;
    }
    else{
        
        return true;
    }
    
}

function floor_collision(c){
    
    pt_velocity(c.a, bodies[c.a].vertexes[c.interpenetratingVertex],padot);
    tempEdge[0]=0;
    tempEdge[1]=1;
    tempEdge[2]=0;
    vrel = dotproduct(tempEdge, padot);
    vectorminus(bodies[c.a].vertexes[c.interpenetratingVertex], bodies[c.a].x, ra);
    numerator = -(1+epsilon)*vrel;
    term1 = 1/(bodies[c.a].mass);
    term2 = 0;
    
    crossproduct(ra, tempEdge, tempArray);
    crossproduct(numeric.dot(bodies[c.a].Iinv,tempArray),ra, tempArray);
    term3 = dotproduct(tempEdge, tempArray);
    
    j = numerator/(term1+term2+term3);
    force[0] = j*tempEdge[0];
    force[1] = j*tempEdge[1];
    force[2] = 0;
    
    bodies[c.a].P[0] = bodies[c.a].P[0] + force[0];
    bodies[c.a].P[1] = bodies[c.a].P[1] + force[1];
    
    crossproduct(ra, force, tempArray);
    bodies[c.a].L[0] = bodies[c.a].L[0] + tempArray[0];
    bodies[c.a].L[1] = bodies[c.a].L[1] + tempArray[1];
    bodies[c.a].L[2] = bodies[c.a].L[2] + tempArray[2];
    
    bodies[c.a].v[0] = bodies[c.a].P[0] / bodies[c.a].mass;
    bodies[c.a].v[1] = bodies[c.a].P[1] / bodies[c.a].mass;
    
    bodies[c.a].omega = numeric.dot(bodies[c.a].Iinv, bodies[c.a].L );
    
    
}

function detectFloorInterpenetratingVertex(a){
    
    for (var k=0;k<4;k++){
        tempEdge3[0] = bodies[a].vertexes[k][0] + 20;
        tempEdge3[1] = bodies[a].vertexes[k][1] + 10;
        tempEdge3[2] = 0;
        crossproduct(floorEdge, tempEdge3, crossproductResult);
        if (crossproductResult[2]>0){
            vorzeichen2 = 1;
        }
        else{
            vorzeichen2 = -1;
        }
        if (vorzeichen2==floorvorzeichen){
            return k;
        }
    }
    return -1;
    
}

all_floor_collisions = new Array(100);

function initFloorCollision(a){
    
    tempInterpenetratingFloorVertex =  detectFloorInterpenetratingVertex(a);
    
    
    floorContact = new FloorContact();
    floorContact.a=a;
    floorContact.interpenetratingVertex = tempInterpenetratingFloorVertex;
    
    all_floor_collisions[floorCollisionCount] = floorContact;
    
}

function checkIfBodyHitFloor(a){
    validCounter=0;
    for (var k=0;k<4;k++){
        tempEdge3[0] = bodies[a].vertexes[k][0] + 20;
        tempEdge3[1] = bodies[a].vertexes[k][1] + 10;
        tempEdge3[2] = 0;
        crossproduct(floorEdge, tempEdge3, crossproductResult);
        if (crossproductResult[2]>0){
            vorzeichen2 = 1;
        }
        else{
            vorzeichen2 = -1;
        }
        if (floorvorzeichen!=vorzeichen2){
            
            validCounter++;
            
        }
    }
    
    
    if (validCounter==4){
        return false;
    }
    
    
    
    return true;
    
    
}

var jacounter=0;

function floor_collisions(){
    
    var had_collision;
    
    jacounter=0;
    floorCollisionCount=0;
    
    for (var i=0;i<numberBodies;i++){
        if (checkIfBodyHitFloor(i)){
                initFloorCollision(i);
                floorCollisionCount++;
        }
    }
    
    for (var i=0;i<floorCollisionCount;i++){
        
        if (floor_colliding(all_floor_collisions[i])){
            floor_collision(all_floor_collisions[i]);
            jacounter++;
        }
        
    }
    

    
}
var called = false;
var called_twice = false;
var called=0;
function collisionDetection(){
    
    update_bounding_boxes_and_find_collisions();
    handle_all_collisions();
    
    collisionCount=0;
    
    bodies_to_array(xFinal);
    
    
    
}