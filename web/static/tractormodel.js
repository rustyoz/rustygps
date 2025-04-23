// Create simple tractor representation (for now just a box)
function tractorModel(wheelbase) {
    // Create a group to hold all tractor parts
    const tractorGroup = new THREE.Group();
    tractorGroup.name = 'tractor';

    // default wheelbase is 3m
    // scale the tractor by the wheelbase
    tractorGroup.scale.set(wheelbase / 3, wheelbase / 3, wheelbase / 3);    

    // Main body dimensions - adjusted width to match cab
    const bodyGeometry = new THREE.BoxGeometry(1, 1.5, 4);
    const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x2e8b57 }); // Sea green
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.name = 'tractor-body';
    body.position.set(0, 1.5, 1.5);
    tractorGroup.add(body);

    // Cab dimensions
    const cabGeometry = new THREE.BoxGeometry(1.5, 1.5, 2);
    const cabMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 }); // Dark gray
    const cab = new THREE.Mesh(cabGeometry, cabMaterial);
    cab.name = 'tractor-cab';
    cab.position.set(0, 2.25, 0.5); // Position cab on top of body, slightly back
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
    frontLeftWheelGroup.position.set(-1.2, frontWheelRadius, 3);
    frontRightWheelGroup.position.set(1.2, frontWheelRadius, 3);
    tractorGroup.add(frontLeftWheelGroup);
    tractorGroup.add(frontRightWheelGroup);

    // Create front wheels within their groups
    const frontLeftWheel = new THREE.Mesh(frontWheelGeometry, wheelMaterial);
    const frontRightWheel = new THREE.Mesh(frontWheelGeometry, wheelMaterial);
    frontLeftWheel.name = 'front-left-wheel';
    frontRightWheel.name = 'front-right-wheel';
    frontLeftWheel.rotation.z = Math.PI / 2;
    frontRightWheel.rotation.z = Math.PI / 2;
    frontLeftWheelGroup.add(frontLeftWheel);
    frontRightWheelGroup.add(frontRightWheel);

    // Create and position rear wheels
    const rearLeftWheel = new THREE.Mesh(rearWheelGeometry, wheelMaterial);
    const rearRightWheel = new THREE.Mesh(rearWheelGeometry, wheelMaterial);
    rearLeftWheel.name = 'rear-left-wheel';
    rearRightWheel.name = 'rear-right-wheel';
    rearLeftWheel.position.set(-1.2, rearWheelRadius, 0);
    rearRightWheel.position.set(1.2, rearWheelRadius, 0);
    rearLeftWheel.rotation.z = Math.PI / 2;
    rearRightWheel.rotation.z = Math.PI / 2;
    tractorGroup.add(rearLeftWheel);
    tractorGroup.add(rearRightWheel);

    // Add axles - adjusted for different wheel heights
    const axleGeometry = new THREE.CylinderGeometry(0.1, 0.1, 2.4, 8);
    const axleMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 }); // Gray

    // Front axle
    const frontAxle = new THREE.Mesh(axleGeometry, axleMaterial);
    frontAxle.name = 'front-axle';
    frontAxle.position.set(0, frontWheelRadius, 3);
    frontAxle.rotation.z = Math.PI / 2;
    tractorGroup.add(frontAxle);

    // Back axle - positioned higher for larger wheels
    const backAxle = new THREE.Mesh(axleGeometry, axleMaterial);
    backAxle.name = 'rear-axle';
    backAxle.position.set(0, rearWheelRadius, 0);
    backAxle.rotation.z = Math.PI / 2;
    tractorGroup.add(backAxle);

    // rotate the tractor by 90 degrees around the y axis anticlockwise
    tractorGroup.rotation.y = Math.PI / 2;

    // Add method to update steering angle
    tractorGroup.setSteeringAngle = function(angleInDegrees) {
        // negate the angle, so that left is negative and right is positive
        const angleInRadians = -(angleInDegrees * Math.PI) / 180;
        frontLeftWheelGroup.rotation.y = angleInRadians;
        frontRightWheelGroup.rotation.y = angleInRadians;
    };

    return tractorGroup;
}