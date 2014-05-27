
use <utils.scad>

// Animatronic eye-pair assembly

// Example:
//eyesAssembly();

rotate([-90, 0, 0])
    eyeAssembly(60);

module eyesAssembly(eyeDiam = 60, eyeDistance = 80) {

    
    translate([-eyeDistance/2, 0, 0])
        eyeAssembly(eyeDiam);

    translate([eyeDistance/2, 0, 0])
        eyeAssembly(eyeDiam);

}



module eyeAssembly(eyeDiam = 20) {

    depth = 0;

    frontCutOffPlane = 0.32;
    backCutOffPlane = 0.65;

    jointPinDiam = 3.1;
    jointGap = 1;
    jointCenterDiam = eyeDiam / 5;
    jointCenterHeight = eyeDiam / 2;

    supportArmW = 5; 
    supportArmH = 10; 
    supportDistance = eyeDiam/2;
    supportPlatformLen = 30;

    translate([0, -depth, 0]) {
        //eyeBase(eyeDiam, frontCutOffPlane = frontCutOffPlane, backCutOffPlane = backCutOffPlane, jointPinDiam = jointPinDiam, jointCenterDiam=jointCenterDiam, jointCenterHeight=jointCenterHeight, cameraPreview=true, jointGap = jointGap);
        eyeJoint(jointPinDiam, jointCenterDiam, jointCenterHeight=jointCenterHeight);
        //eyeHolder(jointPinDiam, jointCenterDiam, jointGap, supportArmW, supportArmH, supportDistance, supportPlatformLen);
        //%eye(eyeDiam, cutOffPlane = frontCutOffPlane);
    }

}



module eyeBase(eyeDiam=20, thickness = 4, frontCutOffPlane = 0.35, backCutOffPlane = 0.65, jointPinDiam = 3, jointCenterDiam = 6, jointCenterHeight=6, spherical = true, cameraMountingHoleDist = 30, cameraMountingHoleDiam = 3, cameraPreview=false, jointGap = 0.5) {
       
    smoothness = 60;

    epsilon = 0.01;
    r = eyeDiam / 2;
    
    tabHoleDiam = 3;

    jointConnectorDiam = jointPinDiam*3;
    jointConnectorH = eyeDiam/2 - jointCenterHeight/2 - jointGap - thickness/2;
    

    frontCutoffY = -(r*2*(1-frontCutOffPlane) - r);
    backCutoffY = r*2*backCutOffPlane - r;
    h =  backCutoffY - frontCutoffY;

    // Ring
    jointConnectorAngles = [0, 180];
    difference() {
        // Ring
        if (spherical) 
            sphere(r=r, $fn=smoothness);   
        else
            translate([0, frontCutoffY+h, 0])
                rotate([90, 0, 0])
                    cylinder(r=r, h=h, $fn=smoothness);

        // Cutout hole in ring                    
        translate([0, frontCutoffY+h+1, 0])
            rotate([90, 0, 0])
                cylinder(r=r- thickness, h=h+2,$fn=smoothness/2);
        translate([-r-epsilon, backCutoffY, -r-epsilon])
            cube([eyeDiam+epsilon*2, eyeDiam, eyeDiam+epsilon*2]);
        translate([-r-epsilon, frontCutoffY-eyeDiam, -r-epsilon])
            cube([eyeDiam+epsilon*2, eyeDiam, eyeDiam+epsilon*2]);

        // Cut holes for universal coint axis    
        for (a=jointConnectorAngles) {
            rotate([0, a, 0])
                translate([0, 0, -eyeDiam/2 + thickness/2])
                    translate([0,0,-thickness/2-1])
                        cylinder(r=jointPinDiam/2, h = jointConnectorH+thickness+2, $fn=60);
        }
    }

    // Joint connector
    for (a=jointConnectorAngles) {
        rotate([0, a, 0])
            translate([0, 0, -eyeDiam/2 + thickness/2])
                difference() {
                    union() {
                        // Connector body
                        cylinder(r=jointConnectorDiam/2, h = jointConnectorH, $fn=30);
                        
                        // Add cube towards front, to make it easier to print
                        translate([0, frontCutoffY/2, jointConnectorH/2+thickness/4])
                            cube([jointConnectorDiam, -frontCutoffY, jointConnectorH-thickness/2], center=true);
                    }                        
                    
                    // Drill axis hole
                    translate([0,0,-thickness/2-1])
                        cylinder(r=jointPinDiam/2, h = jointConnectorH+thickness+2, $fn=60);
                }
    }

    // Tilt actuator tabs
    tiltTabW = tabHoleDiam*2+1;
    tiltTabLen = tiltTabW*1;
    intersection() {    
        for (a = [0, 90, 180, 270]) {
            difference() {
                // Add tilted tab, so that it's easier to print
                rotate([0, a, 0])
                    translate([r-thickness/2-tabHoleDiam*2, backCutoffY-tiltTabW/2, 0])    
                        tabWithHole(tiltTabW, tabHoleDiam, tiltTabLen, thickness, rotateX=0, rotateZ=140, centerX=true, centerZ = true);

                // Cut off anything overlapping the central axis hole
                cylinder(r = jointPinDiam/2 + 1, h=eyeDiam, center=true, $fn=10);    
            }        
        }
        
        // Cut off excess tab length by intersecting with eyeball
        sphere(r=r, $fn=smoothness);        
    }
    
    
    
    // Camera mount tabs
    mountingTabDiam = cameraMountingHoleDiam*2+1;
    cameraHoleDist = sqrt(2) * cameraMountingHoleDist/2;
    cameraMountTabLen = r - cameraHoleDist - mountingTabDiam/2 - thickness/2;
    intersection() {    
        for (a = [45 : 90 : 360-45]) 
            rotate([0, a, 0])
            translate([cameraHoleDist, frontCutoffY+thickness, 0])  
                tabWithHole(mountingTabDiam, cameraMountingHoleDiam, cameraMountTabLen, thickness, rotateX=90, rotateY=180, centerX = true, centerZ = false);

        // Cut off excess tab length by intersecting with eyeball
        sphere(r=r, $fn=smoothness);
    }

    // Camera preview
    if (cameraPreview) {
        translate([0,frontCutoffY,0])
            rotate([90, 0, 0])
                cameraBoard(holeDist = cameraMountingHoleDist, holeDiam = cameraMountingHoleDiam);
    }

}


module eyeJoint(jointPinDiam = 3, jointDiam = 6, jointCenterHeight=6, nutHoleWidth=6, nutHoleHeight=4.5) {

    difference() {
        // Central body
        union() {
            cylinder(r= jointDiam/2, h = jointCenterHeight, center=true, $fn=60);
            rotate([0, 90, 0])
                cylinder(r= jointDiam/2, h = jointDiam, center=true, $fn=60);
             
            // Merge a cube on the back to make it easier to print (and a bit stronger)
            translate([0, jointDiam/2/2, 0])
                cube([jointDiam, jointDiam/2, jointCenterHeight], center= true);                
        }
            
        // Cut holes for axes   
        for (a = [0, 90]) 
            rotate([0, a, 0])
                cylinder(r= jointPinDiam/2, h = max(jointCenterHeight,jointDiam)+2, center=true, $fn=40);
                
        // Cut holes for nuts
        for (a = [0, 180]) 
            rotate([0, a, 0])
                translate([-nutHoleWidth/2, -jointDiam + nutHoleWidth*0.9, jointPinDiam * 1.5])
                    cube([nutHoleWidth, jointDiam, nutHoleHeight]);
                
    }   
}


module eyeHolder(jointPinDiam = 3, jointDiam = 6, jointGap = 0.5, supportArmW=5, supportArmH=5, supportDistance = 30, supportPlatformLen = 20) {

    // Arms holding the eye joint
    envelopeSphereR = jointDiam/2 + jointGap + supportArmW*1.1;
    for (a = [0, 180]) 
        rotate([0, a, 0]) {
            difference() {
                union() {          
                    // Round ends of holders, so that they dont catch on the camera board   
                    intersection() {
                        sphere(r = envelopeSphereR, $fn=60);
                        translate([jointDiam/2 + jointGap, -jointDiam/2, -supportArmH/2])
                            cube([supportArmW, jointDiam/2 + 0.1, supportArmH]);
                    }           
                    translate([jointDiam/2 + jointGap, 0, -supportArmH/2])
                        cube([supportArmW, supportDistance + jointDiam/2, supportArmH]);
                }   
                
                // Cut hole for axis 
                rotate([0, 90, 0])
                    cylinder(r = jointPinDiam/2, h = envelopeSphereR*2+2, center=true, $fn=30);
            }
        }
        
    // Platform
    platformW = jointDiam + 2*jointGap + 2*supportArmW;
    translate([-platformW/2, supportDistance, -supportArmH/2])
        cube([platformW, supportPlatformLen, supportArmH]);    
        
}



module eye(eyeDiam = 20, eyeThickness = 3, pupilFactor = 0.2, iris = 0.3, cutOffPlane = 0.5) {

    smoothness = 100;

    epsilon = 1;
    r = eyeDiam / 2;
    
    pupilR = r * pupilFactor;
    irisR = r * iris;

    color([0.7, 0.7, 0.7]) 
        difference() {
            sphere(r, $fn=smoothness);
            sphere(r - eyeThickness, $fn=smoothness/2);
            translate([-r-epsilon, r*2*cutOffPlane - r, -r-epsilon])
                cube([eyeDiam+epsilon*2, eyeDiam+epsilon*2, eyeDiam+epsilon*2]);
            rotate([90, 0, 0])     
                cylinder(r=pupilR, h=eyeDiam+epsilon, $fn=smoothness);    
            translate([0, -r*0.85, 0])
                rotate([90, 0, 0])     
                    cylinder(r1= 0, r2=irisR*4, h=r*0.15, $fn=smoothness);    
        }

}



module cameraBoard(w = 38, d = 38, h = 20, lensDiam = 12, holeDist = 30, holeDiam = 3) {
    
    pcbT = 1;
    
    holeDistX = holeDist;
    holeDistY = holeDist;

    difference() {    
        color([0.1, 0.6, 0.1])
            translate([-w/2, -d/2, 0])
                cube([w, d, pcbT]);
        for (x = [-holeDistX/2, holeDistX/2])
            for (y = [-holeDistY/2, holeDistY/2])
                translate([x, y, pcbT/2])
                    cylinder(r=holeDiam/2, h=pcbT+2, $fn=30, center=true);
    }        
    color([0.2, 0.2, 0.2])
        translate([0, 0, pcbT])
            cylinder(r=lensDiam/2, h = h-pcbT, $fn=40);

}


