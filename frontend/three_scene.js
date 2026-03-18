class ThreeScene {
    constructor(containerId, onTargetMove, onCollision) {
        this.container = document.getElementById(containerId);
        this.onTargetMove = onTargetMove;
        this.onCollision = onCollision;
        
        // Setup Three.js
        this.scene = new THREE.Scene();
        this.scene.fog = new THREE.FogExp2(0x0f172a, 0.02);

        this.camera = new THREE.PerspectiveCamera(45, this.container.clientWidth / this.container.clientHeight, 0.1, 1000);
        this.camera.up.set(0, 0, 1); // Z is up
        this.camera.position.set(10, 15, 8);

        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);

        // Orbit Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.maxPolarAngle = Math.PI / 2 + 0.2; // Allow slightly below ground

        // Lights
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(10, 20, 10);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        this.scene.add(dirLight);

        const pointLight = new THREE.PointLight(0x38bdf8, 1, 50);
        pointLight.position.set(-5, 5, -5);
        this.scene.add(pointLight);

        // Environment Helpers
        const grid = new THREE.GridHelper(20, 20, 0x38bdf8, 0x475569);
        grid.material.opacity = 0.3;
        grid.material.transparent = true;
        grid.rotation.x = Math.PI / 2; // Make grid lay on XY plane (Z is up)
        this.scene.add(grid);

        const axesHelper = new THREE.AxesHelper(5);
        this.scene.add(axesHelper);
        
        this.buildBase();

        // Arm Objects
        this.joints = [];
        this.links = [];
        this.axes = [];
        this.obstacles = [];
        
        // Ghost Arm
        this.ghostJoints = [];
        this.ghostLinks = [];
        this.showGhost = false;

        // Workspace Cloud
        this.workspacePoints = null;
        
        // IK Target Sphere
        const targetGeo = new THREE.SphereGeometry(0.3, 32, 32);
        const targetMat = new THREE.MeshPhongMaterial({ 
            color: 0xef4444, 
            emissive: 0xef4444, 
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.8
        });
        this.targetSphere = new THREE.Mesh(targetGeo, targetMat);
        this.targetSphere.position.set(3, 3, 3);
        this.targetSphere.visible = false; // Hidden by default (FK mode)
        this.scene.add(this.targetSphere);

        // Path Trail
        this.trailPositions = [];
        this.maxTrailLength = 100;
        const trailMat = new THREE.LineBasicMaterial({ color: 0x10b981, linewidth: 2 });
        const trailGeo = new THREE.BufferGeometry();
        this.trailLine = new THREE.Line(trailGeo, trailMat);
        this.scene.add(this.trailLine);

        // Raycasting for dragging target
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.isDragging = false;
        this.dragPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0); // Z is up normal
        this.dragObject = null;
        
        this.setupEvents();
        
        // Start Render Loop
        this.animate = this.animate.bind(this);
        requestAnimationFrame(this.animate);
    }

    setupEvents() {
        window.addEventListener('resize', () => {
            this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        });

        // Drag events for target & obstacles
        this.renderer.domElement.addEventListener('pointerdown', (e) => {
            const rect = this.renderer.domElement.getBoundingClientRect();
            this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
            
            this.raycaster.setFromCamera(this.mouse, this.camera);
            
            const interactables = [...this.obstacles];
            if (this.targetSphere.visible) interactables.push(this.targetSphere);
            
            const intersects = this.raycaster.intersectObjects(interactables);
            
            if (intersects.length > 0) {
                this.isDragging = true;
                this.dragObject = intersects[0].object;
                this.controls.enabled = false; // Disable orbit while dragging
                
                // Update drag plane to be perpendicular to camera
                this.dragPlane.setFromNormalAndCoplanarPoint(
                    this.camera.getWorldDirection(new THREE.Vector3()).negate(),
                    this.dragObject.position
                );
            }
        });

        window.addEventListener('pointermove', (e) => {
            if (!this.isDragging || !this.dragObject) return;
            
            const rect = this.renderer.domElement.getBoundingClientRect();
            this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
            
            this.raycaster.setFromCamera(this.mouse, this.camera);
            const targetPos = new THREE.Vector3();
            this.raycaster.ray.intersectPlane(this.dragPlane, targetPos);
            
            if (targetPos) {
                // Ensure Z is above ground
                targetPos.z = Math.max(0, targetPos.z);
                this.dragObject.position.copy(targetPos);
                
                if (this.dragObject === this.targetSphere && this.onTargetMove) {
                    this.onTargetMove(targetPos.x, targetPos.y, targetPos.z);
                }
            }
        });

        window.addEventListener('pointerup', () => {
            if (this.isDragging) {
                this.isDragging = false;
                this.dragObject = null;
                this.controls.enabled = true;
            }
        });
    }

    setIKMode(enabled) {
        this.targetSphere.visible = enabled;
    }

    setTargetPosition(x, y, z) {
        this.targetSphere.position.set(x, y, z);
    }

    buildBase() {
        const baseGeo = new THREE.CylinderGeometry(1.5, 1.8, 0.5, 32);
        baseGeo.rotateX(Math.PI / 2); // Align with Z up
        const baseMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.3 });
        const baseMesh = new THREE.Mesh(baseGeo, baseMat);
        baseMesh.position.set(0, 0, 0.25); // Rest on Z=0
        baseMesh.castShadow = true;
        baseMesh.receiveShadow = true;
        this.scene.add(baseMesh);
    }

    createGripper() {
        const group = new THREE.Group();
        const baseGeo = new THREE.BoxGeometry(0.8, 0.8, 0.2);
        const mat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.2 });
        const base = new THREE.Mesh(baseGeo, mat);
        group.add(base);
        
        const fingerGeo = new THREE.BoxGeometry(0.1, 0.6, 0.4);
        const f1 = new THREE.Mesh(fingerGeo, mat);
        f1.position.set(-0.35, 0.3, 0.3);
        group.add(f1);
        
        const f2 = new THREE.Mesh(fingerGeo, mat);
        f2.position.set(0.35, -0.3, 0.3); // opposite sides
        group.add(f2);
        
        return group;
    }

    createBall(x, y, z) {
        const geo = new THREE.SphereGeometry(0.3, 32, 32);
        const mat = new THREE.MeshStandardMaterial({ color: 0xef4444, metalness: 0.3, roughness: 0.5 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.set(x, y, z);
        mesh.castShadow = true;
        this.scene.add(mesh);
        return mesh;
    }

    createBox(x, y, z) {
        const geo = new THREE.BoxGeometry(1.2, 1.2, 0.8);
        const mat = new THREE.MeshStandardMaterial({ color: 0x10b981, metalness: 0.2, roughness: 0.8 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.set(x, y, z);
        mesh.castShadow = true;
        this.scene.add(mesh);
        // Exclude drop box from blocking the effector, keep it just visual for place
        return mesh;
    }

    addObstacle() {
        const obsGeo = new THREE.BoxGeometry(1, 1, 1);
        const obsMat = new THREE.MeshStandardMaterial({ color: 0xeab308, metalness: 0.2, roughness: 0.8 });
        const obsMesh = new THREE.Mesh(obsGeo, obsMat);
        obsMesh.position.set((Math.random() - 0.5) * 8, (Math.random() - 0.5) * 8, 0.5);
        obsMesh.castShadow = true;
        obsMesh.receiveShadow = true;
        this.scene.add(obsMesh);
        this.obstacles.push(obsMesh);
    }

    buildArm(dof) {
        // Clear old arm
        this.joints.forEach(j => this.scene.remove(j));
        this.links.forEach(l => this.scene.remove(l));
        this.axes.forEach(a => this.scene.remove(a));
        
        this.ghostJoints.forEach(j => this.scene.remove(j));
        this.ghostLinks.forEach(l => this.scene.remove(l));
        
        this.joints = [];
        this.links = [];
        this.axes = [];
        this.ghostJoints = [];
        this.ghostLinks = [];

        const jointGeo = new THREE.CylinderGeometry(0.4, 0.4, 0.8, 32);
        jointGeo.rotateX(Math.PI / 2); // Align with Z axis for standard DH
        
        const jointMat = new THREE.MeshStandardMaterial({ color: 0x3b82f6, metalness: 0.6, roughness: 0.2 });
        const linkGeo = new THREE.CylinderGeometry(0.2, 0.2, 1, 16);
        const linkMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.3, roughness: 0.4 });

        const ghostJointMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.2, wireframe: true });
        const ghostLinkMat = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.1, wireframe: true });

        // N joints (Base + N-1 intermediate + End Effector)
        for (let i = 0; i <= dof; i++) {
            let jMesh, gMesh;
            if (i === dof) {
                jMesh = this.createGripper();
                gMesh = this.createGripper(true); // ghost gripper not fully supported visually, use box
            } else {
                jMesh = new THREE.Mesh(jointGeo, jointMat);
                gMesh = new THREE.Mesh(jointGeo, ghostJointMat);
            }
            jMesh.castShadow = true; jMesh.receiveShadow = true;
            this.scene.add(jMesh);
            this.joints.push(jMesh);
            
            gMesh.visible = this.showGhost;
            this.scene.add(gMesh);
            this.ghostJoints.push(gMesh);
            
            const ax = new THREE.AxesHelper(1);
            this.scene.add(ax);
            this.axes.push(ax);

            if (i < dof) {
                const lMesh = new THREE.Mesh(linkGeo, linkMat);
                lMesh.castShadow = true; lMesh.receiveShadow = true;
                this.scene.add(lMesh);
                this.links.push(lMesh);
                
                const glMesh = new THREE.Mesh(linkGeo, ghostLinkMat);
                glMesh.visible = this.showGhost;
                this.scene.add(glMesh);
                this.ghostLinks.push(glMesh);
            }
        }
        
        // Reset trail
        this.trailPositions = [];
        this.trailLine.geometry.setDrawRange(0, 0);
    }

    setGhostVisible(visible) {
        this.showGhost = visible;
        this.ghostJoints.forEach(m => m.visible = visible);
        this.ghostLinks.forEach(m => m.visible = visible);
    }

    updateArm(framesData) {
        // framesData is an array of 4x4 matrix row lists
        // framesData.length should be == this.joints.length
        
        const positions = [];

        for (let i = 0; i < framesData.length; i++) {
            if (i >= this.joints.length) break;

            const r = framesData[i];
            const mat = new THREE.Matrix4();
            mat.set(
                r[0][0], r[0][1], r[0][2], r[0][3],
                r[1][0], r[1][1], r[1][2], r[1][3],
                r[2][0], r[2][1], r[2][2], r[2][3],
                r[3][0], r[3][1], r[3][2], r[3][3]
            );

            // Decompose into pos, quat, scale
            const pos = new THREE.Vector3();
            const quat = new THREE.Quaternion();
            const scale = new THREE.Vector3();
            mat.decompose(pos, quat, scale);

            positions.push(pos);

            this.joints[i].position.copy(pos);
            this.joints[i].quaternion.copy(quat);
            
            this.axes[i].position.copy(pos);
            this.axes[i].quaternion.copy(quat);

            // End effector trail & orientation
            if (i === framesData.length - 1) {
                this.updateTrail(pos);
                
                const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ');
                const roll = THREE.MathUtils.radToDeg(euler.x);
                const pitch = THREE.MathUtils.radToDeg(euler.y);
                const yaw = THREE.MathUtils.radToDeg(euler.z);

                const elRoll = document.getElementById('eff-roll');
                const elPitch = document.getElementById('eff-pitch');
                const elYaw = document.getElementById('eff-yaw');
                
                if (elRoll) elRoll.innerText = roll.toFixed(2) + '°';
                if (elPitch) elPitch.innerText = pitch.toFixed(2) + '°';
                if (elYaw) elYaw.innerText = yaw.toFixed(2) + '°';

                document.getElementById('eff-x').innerText = pos.x.toFixed(2);
                document.getElementById('eff-y').innerText = pos.y.toFixed(2);
                document.getElementById('eff-z').innerText = pos.z.toFixed(2);
            }
        }

        // Update links
        for (let i = 0; i < this.links.length; i++) {
            if (i + 1 >= positions.length) break;
            
            const p1 = positions[i];
            const p2 = positions[i + 1];
            
            const dist = p1.distanceTo(p2);
            if (dist < 0.001) {
                this.links[i].visible = false;
                continue;
            }
            
            this.links[i].visible = true;
            this.links[i].scale.set(1, dist, 1);
            
            // Midpoint
            this.links[i].position.copy(p1).lerp(p2, 0.5);
            
            // Look direction (Cylinder goes along Y, but we have Z-up camera? Actually standard CylinderGeometry goes along Y in local space).
            // Let's orient the cylinder to point from p1 to p2.
            const dir = new THREE.Vector3().subVectors(p2, p1).normalize();
            this.links[i].quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        }
    }

    updateTrail(newPos) {
        this.trailPositions.push(newPos.clone());
        if (this.trailPositions.length > this.maxTrailLength) {
            this.trailPositions.shift();
        }
        
        const pts = [];
        this.trailPositions.forEach(p => pts.push(p.x, p.y, p.z));
        
        const f32 = new Float32Array(pts);
        this.trailLine.geometry.setAttribute('position', new THREE.BufferAttribute(f32, 3));
        this.trailLine.geometry.setDrawRange(0, this.trailPositions.length);
    }

    checkCollisions(positions) {
        if (!positions || positions.length === 0) return false;
        
        let isColliding = false;
        
        // Approximate obstacles as bounding spheres
        const obsSpheres = this.obstacles.map(o => ({
            center: o.position,
            radius: 0.8 // half diagonal of 1x1x1 box is approx 0.866
        }));
        
        const line = new THREE.Line3();
        const closestPoint = new THREE.Vector3();
        
        // Check structural links against spheres
        for (let i = 0; i < positions.length - 1; i++) {
            const p1 = positions[i];
            const p2 = positions[i + 1];
            line.set(p1, p2);
            
            let linkHit = false;
            for (let obs of obsSpheres) {
                line.closestPointToPoint(obs.center, true, closestPoint);
                const dist = closestPoint.distanceTo(obs.center);
                if (dist < obs.radius + 0.2) { // 0.2 is cylinder radius
                    linkHit = true;
                    isColliding = true;
                    break;
                }
            }
            
            // Emissive color warning
            if (this.links && this.links[i] && this.links[i].material) {
                if (linkHit) {
                    this.links[i].material.emissive.setHex(0xff0000);
                    this.links[i].material.emissiveIntensity = 0.8;
                } else {
                    this.links[i].material.emissive.setHex(0x000000);
                    this.links[i].material.emissiveIntensity = 0;
                }
            }
        }
        
        // Check joints
        for (let i = 0; i < positions.length; i++) {
            const p = positions[i];
            let jointHit = false;
            for (let obs of obsSpheres) {
                if (p.distanceTo(obs.center) < obs.radius + 0.4) {
                    jointHit = true;
                    isColliding = true;
                    break;
                }
            }
            if (this.joints && this.joints[i]) {
                const matTarget = this.joints[i].type === 'Group' ? this.joints[i].children.map(c => c.material) : [this.joints[i].material];
                matTarget.forEach(m => {
                    if (m) {
                        if (jointHit) { m.emissive.setHex(0xff0000); m.emissiveIntensity = 0.8; }
                        else { m.emissive.setHex(0x000000); m.emissiveIntensity = 0; }
                    }
                });
            }
        }
        
        if (this.onCollision) this.onCollision(isColliding);
        return isColliding;
    }

    animate() {
        requestAnimationFrame(this.animate);
        this.controls.update();
        if (this.targetSphere.visible) {
            // Pulse effect for target
            const time = Date.now() * 0.005;
            this.targetSphere.material.emissiveIntensity = 0.5 + Math.sin(time) * 0.3;
        }
        
        this.renderer.render(this.scene, this.camera);
    }
}
