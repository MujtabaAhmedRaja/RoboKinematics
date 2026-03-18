const defaultConfigs = {
    2: [
        { theta_offset: 0, d: 0, a: 4, alpha: 0 },
        { theta_offset: 0, d: 0, a: 4, alpha: 0 }
    ],
    3: [
        { theta_offset: 0, d: 2.5, a: 0, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 0, a: 3.5, alpha: 0 },
        { theta_offset: 0, d: 0, a: 3, alpha: 0 }
    ],
    4: [
        { theta_offset: 0, d: 2.5, a: 0, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 0, a: 3.5, alpha: 0 },
        { theta_offset: 0, d: 0, a: 2.5, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 2, a: 0, alpha: 0 }
    ],
    5: [
        { theta_offset: 0, d: 2.5, a: 0, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 0, a: 3.5, alpha: 0 },
        { theta_offset: 0, d: 0, a: 2.5, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 2, a: 0, alpha: -Math.PI / 2 },
        { theta_offset: 0, d: 0, a: 2, alpha: 0 }
    ],
    6: [
        { theta_offset: 0, d: 2.5, a: 0, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 0, a: 3.5, alpha: 0 },
        { theta_offset: 0, d: 0, a: 2.5, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 2, a: 0, alpha: -Math.PI / 2 },
        { theta_offset: 0, d: 0, a: 0, alpha: Math.PI / 2 },
        { theta_offset: 0, d: 2, a: 0, alpha: 0 }
    ]
};

const state = {
    dof: 3,
    mode: 'fk', // 'fk' or 'ik'
    dh_params: JSON.parse(JSON.stringify(defaultConfigs[3])),
    joint_angles: [0, Math.PI/4, -Math.PI/2],
    locked_joints: [false, false, false],
    target: { x: 3, y: 0, z: 3 },
    playingPath: false,
    pathTime: 0
};

let ws = null;
let scene3d = null;
let activeEqInput = null; // Track which input is focused for math keys
let pathAnimFrame = null;
let pickBall = null;
let dropBox = null;

const els = {
    statusDot: document.getElementById('backend-status'),
    statusText: document.getElementById('backend-status-text'),
    dofSelect: document.getElementById('dof-select'),
    btnFk: document.getElementById('btn-fk'),
    btnIk: document.getElementById('btn-ik'),
    btnAddObs: document.getElementById('btn-add-obs'),
    slidersWrapper: document.getElementById('sliders-wrapper'),
    inputsWrapper: document.getElementById('inputs-wrapper'),
    ikTargetDisplay: document.getElementById('ik-target-display'),
    effX: document.getElementById('eff-x'),
    effY: document.getElementById('eff-y'),
    effZ: document.getElementById('eff-z'),
    tarX: document.getElementById('ik-in-x'),
    tarY: document.getElementById('ik-in-y'),
    tarZ: document.getElementById('ik-in-z'),
    fadeOverlay: document.getElementById('loading-overlay'),
    
    // Path math
    eqX: document.getElementById('eq-x'),
    eqY: document.getElementById('eq-y'),
    eqZ: document.getElementById('eq-z'),
    btnRunPath: document.getElementById('btn-run-trajectory'),
    eqError: document.getElementById('eq-error')
};

function init() {
    scene3d = new ThreeScene('canvas-container', onTargetMove);
    connectWebSocket();
    setupUIEvents();
    buildUI();
    scene3d.buildArm(state.dof);
    
    // Set initial target visual
    scene3d.setTargetPosition(state.target.x, state.target.y, state.target.z);
    
    checkAuthStatus();
}

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        els.statusDot.className = 'status-dot connected';
        els.statusText.innerText = 'Connected';
        els.fadeOverlay.style.opacity = '0';
        setTimeout(() => els.fadeOverlay.style.display = 'none', 500);
        sendUpdateRequest();
    };

    ws.onclose = () => {
        els.statusDot.className = 'status-dot disconnected';
        els.statusText.innerText = 'Disconnected';
        els.fadeOverlay.style.display = 'block';
        els.fadeOverlay.style.opacity = '1';
        els.fadeOverlay.innerText = 'Lost connection. Retrying...';
        setTimeout(connectWebSocket, 2000);
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.workspace_points) scene3d.updateWorkspaceCloud(data.workspace_points);
        if (data.frames) {
            const posList = data.frames.map(f => new THREE.Vector3(f[0][3], f[1][3], f[2][3]));
            const isColliding = scene3d.checkCollisions(posList);
            
            if (isColliding && !state.hasCollision) {
                showToast("Physics Boundary Reached!", "error");
                state.hasCollision = true;
                
                if (state.last_good_frames && state.last_good_angles) {
                    scene3d.updateArm(state.last_good_frames);
                    state.joint_angles = state.last_good_angles;
                    updateSlidersFromState();
                }
                return;
            }
            
            state.hasCollision = isColliding;
            if (!isColliding) {
                state.last_good_angles = data.angles;
                state.last_good_frames = data.frames;
            }
            scene3d.updateArm(data.frames);
        }
        if (data.mode === 'ik' && data.angles && !state.hasCollision) {
            state.joint_angles = data.angles;
            updateSlidersFromState();
        }
    };
}

function sendUpdateRequest() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({
        dof: state.dof,
        mode: state.mode,
        dh_params: state.dh_params,
        joint_angles: state.joint_angles,
        locked_joints: state.locked_joints,
        target: state.target
    }));
}

function buildUI() {
    // Sliders & Locks
    els.slidersWrapper.innerHTML = '';
    for (let i = 0; i < state.dof; i++) {
        const div = document.createElement('div');
        div.className = 'slider-container';
        const isLocked = state.locked_joints[i];
        
        div.innerHTML = `
            <div class="slider-header">
                <label>Joint ${i + 1} (θ)</label>
                <span class="slider-val" id="val-q${i}">0.00°</span>
                <button class="lock-btn ${isLocked ? 'locked' : ''}" id="lock-q${i}" title="Lock/Unlock Joint">
                    ${isLocked ? '🔒' : '🔓'}
                </button>
            </div>
            <input type="range" id="slider-q${i}" min="-180" max="180" value="0" step="1">
        `;
        els.slidersWrapper.appendChild(div);

        const slider = document.getElementById(`slider-q${i}`);
        const valLabel = document.getElementById(`val-q${i}`);
        const lockBtn = document.getElementById(`lock-q${i}`);
        
        slider.addEventListener('input', (e) => {
            if (state.mode === 'ik' || state.locked_joints[i]) return;
            const deg = parseFloat(e.target.value);
            valLabel.innerText = deg.toFixed(2) + '°';
            state.joint_angles[i] = deg * (Math.PI / 180);
            sendUpdateRequest();
        });

        lockBtn.addEventListener('click', () => {
            state.locked_joints[i] = !state.locked_joints[i];
            lockBtn.innerText = state.locked_joints[i] ? '🔒' : '🔓';
            lockBtn.className = 'lock-btn ' + (state.locked_joints[i] ? 'locked' : '');
            slider.disabled = state.locked_joints[i] || state.mode === 'ik';
            sendUpdateRequest(); // Inform backend of new constraint
        });
    }

    // Link Inputs
    els.inputsWrapper.innerHTML = '';
    for (let i = 0; i < state.dof; i++) {
        const div = document.createElement('div');
        div.className = 'control-group row';
        div.innerHTML = `
            <label>Link ${i + 1} Length (a/d)</label>
            <input type="number" id="input-L${i}" value="${state.dh_params[i].a || state.dh_params[i].d}" step="0.5" min="0">
        `;
        els.inputsWrapper.appendChild(div);

        const input = document.getElementById(`input-L${i}`);
        input.addEventListener('change', (e) => {
            const val = parseFloat(e.target.value);
            if (state.dh_params[i].a > 0 || state.dh_params[i].d === 0) {
                state.dh_params[i].a = val;
            } else {
                state.dh_params[i].d = val;
            }
            sendUpdateRequest();
        });
    }

    updateSlidersFromState();
}

function updateSlidersFromState() {
    for (let i = 0; i < state.dof; i++) {
        const slider = document.getElementById(`slider-q${i}`);
        const valLabel = document.getElementById(`val-q${i}`);
        if(slider && valLabel) {
            const deg = state.joint_angles[i] * (180 / Math.PI);
            slider.value = deg;
            valLabel.innerText = deg.toFixed(2) + '°';
            slider.disabled = (state.mode === 'ik' || state.locked_joints[i]);
        }
    }
}

function setupUIEvents() {
    // Tabs
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
            btn.classList.add('active');
            document.getElementById(btn.dataset.target).classList.add('active');
        });
    });

    els.dofSelect.addEventListener('change', (e) => {
        state.dof = parseInt(e.target.value);
        state.dh_params = JSON.parse(JSON.stringify(defaultConfigs[state.dof]));
        state.joint_angles = Array(state.dof).fill(0);
        state.locked_joints = Array(state.dof).fill(false);
        buildUI();
        scene3d.buildArm(state.dof);
        sendUpdateRequest();
    });

    els.btnFk.addEventListener('click', () => {
        state.mode = 'fk';
        els.btnFk.className = 'active';
        els.btnIk.className = '';
        els.ikTargetDisplay.classList.add('hidden');
        scene3d.setIKMode(false);
        updateSlidersFromState();
        sendUpdateRequest();
    });
    
    document.getElementById('chk-axes').addEventListener('change', (e) => {
        scene3d.axes.forEach(a => { a.visible = e.target.checked; });
    });
    
    document.getElementById('chk-ghost').addEventListener('change', (e) => {
        scene3d.setGhostVisible(e.target.checked);
    });
    
    document.getElementById('btn-export-csv').addEventListener('click', () => {
        let csv = 'Joint,d,a,alpha,theta_offset,angle_rad\n';
        for(let i = 0; i < state.dof; i++) {
           const dh = state.dh_params[i];
           csv += `${i+1},${dh.d},${dh.a},${dh.alpha},${dh.theta_offset},${state.joint_angles[i]}\n`;
        }
        const b = new Blob([csv], { type: 'text/csv' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(b); a.download = 'robot_dh.csv'; a.click();
    });
    
    document.getElementById('btn-gen-workspace').addEventListener('click', () => {
        if (!ws || ws.readyState !== WebSocket.OPEN) return;
        ws.send(JSON.stringify({ dof: state.dof, dh_params: state.dh_params, mode: 'workspace' }));
        showToast('Generating Workspace...', 'success');
    });
    
    document.getElementById('btn-demo-pick').addEventListener('click', () => {
        els.eqX.value = "3";
        els.eqY.value = "4 * Math.cos(state.pathTime)";
        els.eqZ.value = "0.5 + 3 * Math.abs(Math.sin(state.pathTime))"; 
        
        if (!pickBall) pickBall = scene3d.createBall(3, 4, 0.5);
        if (!dropBox) dropBox = scene3d.createBox(3, -4, 0.4);
        
        showToast("Loaded Pick & Place Demo. Click Run!");
    });

    els.btnIk.addEventListener('click', () => {
        state.mode = 'ik';
        els.btnIk.className = 'active';
        els.btnFk.className = '';
        els.ikTargetDisplay.classList.remove('hidden');
        scene3d.setTargetPosition(state.target.x, state.target.y, state.target.z);
        scene3d.setIKMode(true);
        updateSlidersFromState();
        sendUpdateRequest();
    });

    els.btnAddObs.addEventListener('click', () => {
        scene3d.addObstacle();
    });

    // Manual Object Target Inputs
    const updateTargetFromInput = () => {
        state.target.x = parseFloat(els.tarX.value) || 0;
        state.target.y = parseFloat(els.tarY.value) || 0;
        state.target.z = parseFloat(els.tarZ.value) || 0;
        scene3d.setTargetPosition(state.target.x, state.target.y, state.target.z);
        if (state.mode === 'ik') sendUpdateRequest();
    };
    els.tarX.addEventListener('change', updateTargetFromInput);
    els.tarY.addEventListener('change', updateTargetFromInput);
    els.tarZ.addEventListener('change', updateTargetFromInput);

    // Equation Builder keys
    document.querySelectorAll('.math-key').forEach(btn => {
        btn.addEventListener('click', () => {
            if (activeEqInput) {
                activeEqInput.value += btn.innerText;
                activeEqInput.focus();
            } else {
                els.eqX.value += btn.innerText;
            }
        });
    });
    
    [els.eqX, els.eqY, els.eqZ].forEach(el => {
        el.addEventListener('focus', () => activeEqInput = el);
    });

    els.btnRunPath.addEventListener('click', toggleTrajectory);
}

function onTargetMove(x, y, z) {
    if (state.mode === 'ik') {
        state.target.x = x;
        state.target.y = y;
        state.target.z = z;
        els.tarX.value = x.toFixed(2);
        els.tarY.value = y.toFixed(2);
        els.tarZ.value = z.toFixed(2);
        sendUpdateRequest();
    }
}

function toggleTrajectory() {
    if (state.playingPath) {
        state.playingPath = false;
        els.btnRunPath.innerText = 'Run Trajectory Generation';
        els.btnRunPath.classList.remove('running');
        cancelAnimationFrame(pathAnimFrame);
        els.eqError.classList.add('hidden');
        return;
    }

    // Attempt to parse functions
    try {
        const fx = new Function('t', `return ${els.eqX.value}`);
        const fy = new Function('t', `return ${els.eqY.value}`);
        const fz = new Function('t', `return ${els.eqZ.value}`);
        
        // Test evaluation
        fx(0); fy(0); fz(0);
        
        state.playingPath = true;
        state.pathTime = 0;
        els.btnRunPath.innerText = 'Stop Trajectory';
        els.btnRunPath.classList.add('running');
        els.eqError.classList.add('hidden');
        
        // Force IK mode
        if (state.mode !== 'ik') els.btnIk.click();
        
        function loop() {
            if (!state.playingPath) return;
            state.pathTime += 0.05; // Time step
            
            state.target.x = fx(state.pathTime);
            state.target.y = fy(state.pathTime);
            state.target.z = fz(state.pathTime);
            
            els.tarX.value = state.target.x.toFixed(2);
            els.tarY.value = state.target.y.toFixed(2);
            els.tarZ.value = state.target.z.toFixed(2);
            
            scene3d.setTargetPosition(state.target.x, state.target.y, state.target.z);
            sendUpdateRequest();
            
            if (pickBall && dropBox) {
                // Moving forward (4 to -4) happens when sin > 0
                const isCarrying = Math.sin(state.pathTime) > 0;
                if (isCarrying) {
                    // Ball visually bound to the end effector
                    pickBall.position.set(state.target.x, state.target.y, state.target.z - 0.3);
                } else {
                    // Deposited in the drop box
                    pickBall.position.set(3, -4, 0.8);
                    // Cycle repeat: teleports back to pickup zone
                    if (Math.cos(state.pathTime) > 0.95) pickBall.position.set(3, 4, 0.5);
                }
            }
            
            setTimeout(loop, 100);
        }
        loop();
        
    } catch (e) {
        els.eqError.innerText = 'Invalid equation variables/syntax. Use t and Math functions.';
        els.eqError.classList.remove('hidden');
    }
}

// --- SaaS Authentication & API ---

function openModal(id) { document.getElementById(id).classList.remove('hidden'); }
function closeModal(id) { document.getElementById(id).classList.add('hidden'); }

let currentAuthMode = 'login';
function switchAuthTab(mode) {
    currentAuthMode = mode;
    document.getElementById('tab-login-btn').className = mode === 'login' ? 'auth-tab active' : 'auth-tab';
    document.getElementById('tab-register-btn').className = mode === 'register' ? 'auth-tab active' : 'auth-tab';
    const btn = document.getElementById('btn-submit-auth');
    btn.innerText = mode === 'login' ? 'Secure Login' : 'Create Account';
    btn.onclick = mode === 'login' ? submitLogin : submitRegister;
}

function showToast(msg, type='success') {
    const container = document.getElementById('toast-container');
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.innerText = msg;
    container.appendChild(toast);
    setTimeout(() => { toast.remove(); }, 4000);
}

function checkAuthStatus() {
    const token = localStorage.getItem('robo_token');
    const username = localStorage.getItem('robo_user');
    if (token && username) {
        document.getElementById('nav-unauth').classList.add('hidden');
        document.getElementById('nav-auth').classList.remove('hidden');
        document.getElementById('nav-username').innerText = username;
    } else {
        document.getElementById('nav-unauth').classList.remove('hidden');
        document.getElementById('nav-auth').classList.add('hidden');
    }
}

function logout() {
    localStorage.removeItem('robo_token');
    localStorage.removeItem('robo_user');
    checkAuthStatus();
    showToast('Logged out successfully');
}

async function submitLogin() {
    const user = document.getElementById('auth-user').value;
    const pass = document.getElementById('auth-pass').value;
    
    const formData = new FormData();
    formData.append('username', user);
    formData.append('password', pass);
    
    try {
        const res = await fetch('/api/auth/token', { method: 'POST', body: formData });
        const data = await res.json();
        if (res.ok) {
            localStorage.setItem('robo_token', data.access_token);
            localStorage.setItem('robo_user', data.username);
            closeModal('auth-modal');
            checkAuthStatus();
            showToast('Login successful!');
        } else {
            showToast(data.detail || 'Login failed', 'error');
        }
    } catch(e) { showToast('Connection error', 'error'); }
}

async function submitRegister() {
    const user = document.getElementById('auth-user').value;
    const pass = document.getElementById('auth-pass').value;
    try {
        const res = await fetch('/api/auth/register', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({username: user, password: pass})
        });
        const data = await res.json();
        if (res.ok) {
            showToast('Registration successful! Please login.');
        } else {
            showToast(data.detail || 'Registration failed', 'error');
        }
    } catch(e) { showToast('Connection error', 'error'); }
}

function getAuthHeader() {
    return { 'Authorization': `Bearer ${localStorage.getItem('robo_token')}` };
}

async function submitSaveConfig() {
    const title = document.getElementById('save-title').value || 'My Custom Arm';
    try {
        const res = await fetch('/api/configs/', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json', ...getAuthHeader() },
            body: JSON.stringify({
                title: title,
                dof: state.dof,
                dh_params: state.dh_params,
                locked_joints: state.locked_joints
            })
        });
        if (res.ok) {
            showToast('Configuration saved to cloud!');
            closeModal('save-modal');
        } else {
            showToast('Failed to save config. Session expired?', 'error');
        }
    } catch(e) { showToast('Connection error', 'error'); }
}

async function loadConfigs() {
    try {
        const res = await fetch('/api/configs/', { headers: getAuthHeader() });
        if (!res.ok) throw new Error('Unauthorized');
        const configs = await res.json();
        
        const container = document.getElementById('config-list-container');
        container.innerHTML = '';
        if (configs.length === 0) {
            container.innerHTML = '<p>No saved configurations found.</p>';
        } else {
            configs.forEach(c => {
                const div = document.createElement('div');
                div.className = 'config-list-item';
                div.innerHTML = `<div><strong>${c.title}</strong><br><small>${c.dof} DOF Arm</small></div> <button class="nav-btn primary">Load</button>`;
                div.querySelector('button').onclick = () => applyConfig(c);
                container.appendChild(div);
            });
        }
        openModal('load-modal');
    } catch(e) {
        showToast('Please login to load configs.', 'error');
    }
}

function applyConfig(c) {
    state.dof = c.dof;
    state.dh_params = c.dh_params;
    state.locked_joints = c.locked_joints;
    state.joint_angles = Array(c.dof).fill(0);
    els.dofSelect.value = c.dof;
    
    buildUI();
    scene3d.buildArm(state.dof);
    sendUpdateRequest();
    
    closeModal('load-modal');
    showToast(`Loaded ${c.title}`);
}

window.addEventListener('DOMContentLoaded', init);
