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

function vectorlength(v){
    
    var length = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
    length = Math.sqrt(length);
    return length;
}