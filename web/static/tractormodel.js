// Create simple tractor representation (for now just a box)
function tractorModel(wheelbase, hitchoffset) {
    // Create a group to hold all tractor parts
    const tractorGroup = new THREE.Group();
    tractorGroup.name = 'tractor';

    tractorGroup.hitchoffset = hitchoffset;
    // default wheelbase is 3m
    // scale the tractor by the wheelbase
    tractorGroup.scale.set(wheelbase / 3, wheelbase / 3, wheelbase / 3);    

    // Main body dimensions - adjusted width to match cab
    const bodyGeometry = new THREE.BoxGeometry(4, 1.5, 1);
    const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x2e8b57 }); // Sea green
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.name = 'tractor-body';
    body.position.set(1.5, 0, 1.5);
    tractorGroup.add(body);

    // Cab dimensions
    const cabGeometry = new THREE.BoxGeometry(2, 1.5, 1.5);
    const cabMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 }); // Dark gray
    const cab = new THREE.Mesh(cabGeometry, cabMaterial);
    cab.name = 'tractor-cab';
    cab.position.set(0.5, 0, 2.25); // Position cab on top of body, slightly back
    tractorGroup.add(cab);

    // Front wheel dimensions
    const frontWheelRadius = 0.5;
    const frontWheelThickness = 0.4;
    // Rear wheel dimensions - doubled
    const rearWheelRadius = 1.0;
    const rearWheelThickness = 0.6;

    // Create wheel geometries
    const frontWheelGeometry = new THREE.CylinderGeometry(frontWheelRadius, frontWheelRadius, frontWheelThickness, 16);
    const rearWheelGeometry = new THREE.CylinderGeometry(rearWheelRadius, rearWheelRadius, rearWheelThickness, 16);
    const wheelMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 }); // Dark gray

    // Create groups for front wheels to allow steering
    const frontLeftWheelGroup = new THREE.Group();
    const frontRightWheelGroup = new THREE.Group();
    frontLeftWheelGroup.name = 'front-left-wheel-group';
    frontRightWheelGroup.name = 'front-right-wheel-group';
    frontLeftWheelGroup.position.set(3, -1.2, frontWheelRadius);
    frontRightWheelGroup.position.set(3, 1.2, frontWheelRadius);
    tractorGroup.add(frontLeftWheelGroup);
    tractorGroup.add(frontRightWheelGroup);

    // Create front wheels within their groups
    const frontLeftWheel = new THREE.Mesh(frontWheelGeometry, wheelMaterial);
    const frontRightWheel = new THREE.Mesh(frontWheelGeometry, wheelMaterial);
    frontLeftWheel.name = 'front-left-wheel';
    frontRightWheel.name = 'front-right-wheel';

    frontLeftWheelGroup.add(frontLeftWheel);
    frontRightWheelGroup.add(frontRightWheel);

    // Create and position rear wheels
    const rearLeftWheel = new THREE.Mesh(rearWheelGeometry, wheelMaterial);
    const rearRightWheel = new THREE.Mesh(rearWheelGeometry, wheelMaterial);
    rearLeftWheel.name = 'rear-left-wheel';
    rearRightWheel.name = 'rear-right-wheel';
    rearLeftWheel.position.set(0, -1.2, rearWheelRadius);
    rearRightWheel.position.set(0, 1.2, rearWheelRadius);
    tractorGroup.add(rearLeftWheel);
    tractorGroup.add(rearRightWheel);

    // Add axles - adjusted for different wheel heights
    const axleGeometry = new THREE.CylinderGeometry(0.1, 0.1, 2.4, 8);
    const axleMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 }); // Gray

    // Front axle
    const frontAxle = new THREE.Mesh(axleGeometry, axleMaterial);
    frontAxle.name = 'front-axle';
    frontAxle.position.set(3, 0, frontWheelRadius);
    tractorGroup.add(frontAxle);

    // Back axle - positioned higher for larger wheels
    const backAxle = new THREE.Mesh(axleGeometry, axleMaterial);
    backAxle.name = 'rear-axle';
    backAxle.position.set(0, 0, rearWheelRadius);
    tractorGroup.add(backAxle);

    // Add hitch
    const hitchGeometry = new THREE.BoxGeometry(hitchoffset, 0.2, 0.2);
    const hitchMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 }); // Gray
    const hitch = new THREE.Mesh(hitchGeometry, hitchMaterial);
    hitch.name = 'hitch';
    // Position the hitch behind the tractor, centered horizontally and at a reasonable height
    hitch.position.set(-hitchoffset/2, 0, 1);
    tractorGroup.add(hitch);

    

    // Add method to update steering angle
    tractorGroup.setSteeringAngle = function(angleInDegrees) {
        const angleInRadians = (angleInDegrees * Math.PI) / 180;
        frontLeftWheelGroup.rotation.z = angleInRadians;
        frontRightWheelGroup.rotation.z = angleInRadians;
    };



    return tractorGroup;
}

// Add method to add implement to tractor
function addImplement(tractor, implement) {
    // get the hitch offset from the tractor
    const hitchoffset = tractor.hitchoffset;

    // create a group to hold the implement
    const implementGroup = new THREE.Group();
    implementGroup.name = 'implementgroup';
    implementGroup.position.set(-hitchoffset, 0, 0);
    implementGroup.add(implement);
    tractor.add(implementGroup);
    
    // add a method to set the implement angle
    tractor.setImplementAngle = function(angle) {
        implementGroup.rotation.z = angle;
    };
}

function getHitchOffset(tractor) {
    return tractor.hitchoffset;
}


function getImplementAngle(tractor) {
    return tractor.implementgroup.rotation.z;
}