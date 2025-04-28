// Create simple implement representation (a basic toolbar/cultivator style implement)
function implementModel(width, length) {
    // Create an inner group to hold all implement parts
    const implementGroup = new THREE.Group();
    implementGroup.name = 'implement';

    
    // Main hitch beam (center beam that connects to tractor)
    const hitchGeometry = new THREE.BoxGeometry(length, 0.2, 0.2);
    const hitchMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 }); // Gray
    const hitchBeam = new THREE.Mesh(hitchGeometry, hitchMaterial);
    hitchBeam.name = 'implement-hitch';
    hitchBeam.position.set(length/2, 0, 0.5);
    implementGroup.add(hitchBeam);

    // Main toolbar (perpendicular beam that holds the wheels)
    const toolbarGeometry = new THREE.BoxGeometry(2, width, 0.2);
    const toolbarMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 }); // Gray
    const toolbar = new THREE.Mesh(toolbarGeometry, toolbarMaterial);
    toolbar.name = 'implement-toolbar';
    toolbar.position.set(0, 0, 0.5); // Now at the back end of the implement
    implementGroup.add(toolbar);

    // Wheel dimensions
    const wheelRadius = 0.3;
    const wheelThickness = 0.2;

    // Create wheel geometry
    const wheelGeometry = new THREE.CylinderGeometry(wheelRadius, wheelRadius, wheelThickness, 16);
    const wheelMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 }); // Dark gray

    // Create and position left wheel
    const leftWheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
    leftWheel.name = 'implement-left-wheel';
    leftWheel.position.set(0, -width/2 + 0.1, wheelRadius); // Aligned with toolbar, slight offset from edge
    leftWheel.rotation.x = Math.PI / 2;
    implementGroup.add(leftWheel);

    // Create and position right wheel
    const rightWheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
    rightWheel.name = 'implement-right-wheel';
    rightWheel.position.set(0, width/2 - 0.1, wheelRadius); // Aligned with toolbar, slight offset from edge
    rightWheel.rotation.x = Math.PI / 2;
    implementGroup.add(rightWheel);

    // create outer group to hold implement
    const implementOuterGroup = new THREE.Group();
    implementOuterGroup.name = 'implement-outer';
    implementOuterGroup.add(implementGroup);

    // move implement to the left
    implementGroup.position.set(-length, 0, 0);

    return implementOuterGroup;
} 