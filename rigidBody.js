function rigidBody(){
    
    this.mass = 1;
    this.Ibody = new Array(3);
    for (var i = 0; i<3; i++){
        this.Ibody[i] = new Array(3);
    }
    this.Ibodyinv = new Array(3);
    for (var i=0; i<3; i++){
        this.Ibodyinv[i] = new Array(3);
    }
    
    this.x = new Array(3);
    this.R = new Array(3);
    for (var i=0;i<3;i++){
        this.R[i] = new Array(3);
    }
    this.P = new Array(3);
    this.L = new Array(3);
    
    this.Iinv = new Array(3);
    for (var i=0;i<3;i++){
        this.Iinv[i] = new Array(3);
    }
    this.v = new Array(3);
    this.omega = new Array(3);
    
    this.force = new Array(3);
    this.torque = new Array(3);
    
    this.q = new Quaternion();
    
    this.vertexes = new Array(4);
    for (var i=0;i<4;i++){
        this.vertexes[i] = new Array(3);
    }
    
    this.init = function(){
        
        for (var i=0;i<3;i++){
            for (var k=0;k<3;k++){
                this.Ibody[i][k]=0;
                this.Ibodyinv[i][k]=0;
                this.Iinv[i][k]=0;
                this.R[i][k]=0;
            }
        }
        
        this.R[0][0]=1;
        this.R[1][1]=1;
        this.R[2][2]=1;
        
        this.P[0]=0;
        this.P[1]=0;
        this.P[2]=0;
        
        this.L[0]=0;
        this.L[1]=0;
        this.L[2]=0;
        
        this.Ibody[0][0]=1/3;
        this.Ibody[1][1]=1/3;
        this.Ibody[2][2]=2/3;
        
        this.Ibodyinv[0][0] = 3;
        this.Ibodyinv[1][1] = 3;
        this.Ibodyinv[2][2] = 3/2;
        
        this.q.vector[0]=0;
        this.q.vector[1]=0;
        this.q.vector[2]=0;
        this.q.s=1;
        
        this.omega[0]=0;
        this.omega[1]=0;
        this.omega[2]=0;
        
        this.force[0]=0;
        this.force[1]=0;
        this.force[2]=0;
        
        this.torque[0]=0;
        this.torque[1]=0;
        this.torque[2]=0;
        
        this.mass=1;
    }
    


}