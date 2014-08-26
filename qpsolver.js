function coeffs(){
    
    //-1 means in C, 1 means in NC
    this.coeff = -1;
    this.heap = 0;
    
}

var numberOfCoeffs = 0;
var numberOfCoeffsinC = 0;
var numberOfCoeffsinNC = 0;
var allCoeffs;
var forces;
var a;
var deltaf;
var deltaa;
var A;
var temp;
var sj = new Array(2);

function createVectorACd(d){
    
    temp = numberOfCoeffs-numberOfCoeffsinC;
    var tempvector = new Array(numberOfCoeffs);
    var Acd = new Array(temp);
    var counter=0;
    
    for (var i=0;i<numberOfCoeffs;i++){
        
        tempvector[i] = A[i][d];
        
    }
    
    for (var i=0;i<numberOfCoeffs;i++){
        if (allCoeffs[i].heap==-1){
            Acd[counter] = tempvector[i];
            counter++;
        }
    }
    
    return Acd;
    
}

function createMatrixACC(){
    
    temp = numberOfCoeffs-numberOfCoeffsinC;
    var Acc = new Array(temp);
    var rowcounter=0;
    var columncounter=0;
    
    
    for (var i=0; i<temp;i++){
        Acc[i] = new Array(temp);
    }
    
    
    
    for (var i=0;i<numberOfCoeffs;i++){
        columncounter=0;
        if (allCoeffs[i].heap==-1){
            for (var k=0;k<numberOfCoeffs;k++){
                if (allCoeffs[k].heap==-1){
                    Acc[rowcounter][columncounter] = A[i][k];
                    columncounter++;
                }
            
            }
            rowcounter++;
            
        }
    }
    
    return Acc;
    
}

function maxstep(f, a, deltaf, deltaa, d){
    
    var s = Number.POSITIVE_INFINITY;
    var j = -1;
    var stemp;
    if (deltaa[d]>0){
        j = d;
        s = (-1*a[d])/deltaa[d];
    }
    
    for (var i=0;i<numberOfCoeffs;i++){
        if (allCoeffs[i].heap==-1){
            if (deltaf[i]<0){
                stemp = (-1*f[i])/deltaf[i];
                if (stemp<s){
                    s=stemp;
                    j=i;
                }
            }
        }
    }
    
    for (var i=0;i<numberOfCoeffs;i++){
        if (allCoeffs[i].heap==1){
            if (deltaa[i]<0){
                stemp = (-1*a[i])/deltaa[i];
                if (stemp<s){
                    s = stemp;
                    j = i;
                }
            }
        }
    }
    
    sj[0] = s;
    sj[1] = j;
    
    //
}

function fdirection(d){
    
    for (var i=0;i<numberOfCoeffs;i++){
        deltaf[i]=0;
    }
    
    
    deltaf[d]=1;
    
    
    var A11 = createMatrixACC();
    
    var v1 = createVectorACd(d);
    
    for (var i=0;i<numberOfCoeffs-numberOfCoeffsinC;i++){
        v1[i] = v1[i]*-1;
    }
    var x = numeric.solve(A11,v1);
    
    
    for (var i=0;i<numberOfCoeffs;i++){
        if (allCoeffs[i].heap==-1){
            deltaf[i]=x[i];
        }
    }
    
    
}

function drive_to_zeroL1(d){
    
    fdirection(d);
    
    
    
    
    deltaa = numeric.dot(A,deltaf);
   
    //sets sj
    maxstep(forces, a, deltaf, deltaa, d);
    
    //f = f+s*deltaf; a = a+s*deltaa;
    
    
    
    for (var i=0;i<numberOfCoeffs;i++){
        if (deltaf[i]!=0){
            forces[i] = forces[i]+sj[0]*deltaf[i];
        }
        if (deltaa[i]!=0){
            a[i] = a[i]+sj[0]*deltaa[i];
        }
    }
    
}

function drive_to_zero(d){
    
    
    drive_to_zeroL1(d);
    
    
    if (allCoeffs[sj[1]].heap==-1){
        allCoeffs[sj[1]].heap=1;
        drive_to_zeroL1(d);
    }
    else if (allCoeffs[sj[1]].heap==1){
        allCoeffs[sj[1]].heap=-1;
        drive_to_zeroL1(d);
    }
    else{
       allCoeffs[sj[1]].heap=-1;
       return;
    }
    
   
    
}

function find_smaller_zero_coeff(){
    
    for (var i=0;i<a.length;i++){
        if (a[i]<0){
            return i;
        }
    }
    return -1;
    
}


function qp_solve(amat, bvec, fout){
    
    numberOfCoeffs = bvec.length;
    
    allCoeffs = new Array(bvec.length);
    
    for (var i=0;i<bvec.length;i++){
        allCoeffs[i] = new coeffs();
        allCoeffs[i].coeff = i+1;
        allCoeffs[i].heap = 0;
    }
    
    forces = new Array(bvec.length);
    deltaf = new Array(bvec.length);
    deltaa = new Array(bvec.length);
    for (var i=0; i<bvec.length; i++){
        forces[i] = 0;
    }
    
    a = new Array(bvec.length);
    a = bvec.slice(0);
    A = new Array(bvec.length);
    for (var i=0;i<bvec.length;i++){
        A[i] = new Array(bvec.length);
    }
    for (var i=0;i<bvec.length;i++){
        for (var k=0;k<bvec.length;k++){
            A[i][k] = amat[i][k];
        }
    }
    
    var algorithm_end = false;
    var coefftemp;
    var counter=0;
    
    while(!algorithm_end){
        coefftemp = find_smaller_zero_coeff();
        if (coefftemp==-1){
            algorithm_end = true;
        }
        else{
            drive_to_zero(coefftemp);
            counter++;
        }
    }
    
    for (var i=0;i<10;i++){
        coefftemp = find_smaller_zero_coeff();
        if (coefftemp!=-1){
            drive_to_zero(coefftemp);
        }
    }
    
    
    for (var i=0;i<bvec.length;i++){
    fout[i] = forces[i];
    }
}