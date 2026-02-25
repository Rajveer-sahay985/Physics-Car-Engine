#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"     // Required for high-performance Batch Rendering
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

// ==========================================
// 1. MATH HELPERS
// ==========================================
Vector3 Subtract(Vector3 v1, Vector3 v2) { return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z}; }
Vector3 Add(Vector3 v1, Vector3 v2) { return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }
Vector3 Scale(Vector3 v, float s) { return {v.x * s, v.y * s, v.z * s}; }
float Dot(Vector3 v1, Vector3 v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }

Vector3 Normalize(Vector3 v) {
    float length = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length == 0.0f) return {0,0,0};
    return {v.x/length, v.y/length, v.z/length};
}

Vector3 Cross(Vector3 v1, Vector3 v2) {
    return { v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x };
}

float Distance(Vector3 v1, Vector3 v2) {
    Vector3 d = Subtract(v1, v2);
    return std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
}

// ==========================================
// 2. DATA STRUCTURES
// ==========================================
struct Particle {
    Vector3 position;
    Vector3 previous_position;
    Vector3 acceleration;
    float mass;
    bool isWheel;
};

struct Spring {
    Particle* p1;
    Particle* p2;
    float rest_length;
    float original_length; // NEW: Tracks the starting length to cap deformation
    float stiffness;
    float damping; 
    bool isSuspension; 
};

struct VisualVertex {
    Vector3 position;
    int boundParticleIndex; 
    Vector3 offset;         
};

struct Obstacle {
    Vector3 minBounds;
    Vector3 maxBounds;
    Color color;
};

int pFront = 0, pBack = 0, pTop = 0; 

// ==========================================
// 3. PHYSICS BACKEND (Stable V2.3 Core)
// ==========================================
void UpdateParticle(Particle& p, float deltaTime) {
    Vector3 velocity = Subtract(p.position, p.previous_position);
    velocity = Scale(velocity, 0.99f); 

    p.previous_position = p.position;
    p.position.x += velocity.x + p.acceleration.x * deltaTime * deltaTime;
    p.position.y += velocity.y + p.acceleration.y * deltaTime * deltaTime;
    p.position.z += velocity.z + p.acceleration.z * deltaTime * deltaTime;
    p.acceleration = {0.0f, 0.0f, 0.0f};
}

void ApplyTireFriction(Particle& wheel, Vector3 heading, float grip) {
    Vector3 vel = Subtract(wheel.position, wheel.previous_position);
    float forwardSpeed = Dot(vel, heading);
    Vector3 forwardVel = Scale(heading, forwardSpeed);
    
    Vector3 lateralVel = Subtract(vel, forwardVel);
    lateralVel = Scale(lateralVel, 1.0f - grip);
    
    Vector3 newVel = Add(forwardVel, lateralVel);
    wheel.previous_position = Subtract(wheel.position, newVel);
}

void ApplySpringForce(Spring& spring, float deltaTime) {
    float current_distance = Distance(spring.p1->position, spring.p2->position);
    if (current_distance == 0.0f) return;

    Vector3 dir = Normalize(Subtract(spring.p2->position, spring.p1->position));
    float displacement = current_distance - spring.rest_length;

    // REALISTIC CRUMPLE: Metal deforms, but caps at 50% crush to prevent spikes
    if (!spring.isSuspension) {
        float yield_point = 0.3f; // Slightly easier to crumple
        if (std::abs(displacement) > yield_point) {
            float crushAmount = displacement * 0.05f;
            // Only allow the spring to change length if it hasn't crushed past 50%
            if (std::abs(spring.rest_length + crushAmount - spring.original_length) < (spring.original_length * 0.5f)) {
                spring.rest_length += crushAmount; 
            }
        }
    }

    Vector3 v1 = Subtract(spring.p1->position, spring.p1->previous_position);
    Vector3 v2 = Subtract(spring.p2->position, spring.p2->previous_position);
    Vector3 relVel = Subtract(v2, v1);
    float dampForce = Dot(relVel, dir) * spring.damping;

    float force = (spring.stiffness * displacement) + dampForce;
    float acc1 = force / spring.p1->mass;
    float acc2 = force / spring.p2->mass;

    spring.p1->acceleration = Add(spring.p1->acceleration, Scale(dir, acc1));
    spring.p2->acceleration = Subtract(spring.p2->acceleration, Scale(dir, acc2));
}

void HandleObstacleCollision(Particle& p, const Obstacle& obs) {
    if (p.position.x > obs.minBounds.x && p.position.x < obs.maxBounds.x &&
        p.position.y > obs.minBounds.y && p.position.y < obs.maxBounds.y &&
        p.position.z > obs.minBounds.z && p.position.z < obs.maxBounds.z) {
        
        float distToMinX = std::abs(p.position.x - obs.minBounds.x);
        float distToMaxX = std::abs(obs.maxBounds.x - p.position.x);
        float distToMinY = std::abs(p.position.y - obs.minBounds.y);
        float distToMaxY = std::abs(obs.maxBounds.y - p.position.y);
        float distToMinZ = std::abs(p.position.z - obs.minBounds.z);
        float distToMaxZ = std::abs(obs.maxBounds.z - p.position.z);

        float minDist = std::min({distToMinX, distToMaxX, distToMinY, distToMaxY, distToMinZ, distToMaxZ});

        if (minDist == distToMinX) { p.position.x = obs.minBounds.x; p.previous_position.x = p.position.x; }
        else if (minDist == distToMaxX) { p.position.x = obs.maxBounds.x; p.previous_position.x = p.position.x; }
        else if (minDist == distToMinY) { p.position.y = obs.minBounds.y; p.previous_position.y = p.position.y; }
        else if (minDist == distToMaxY) { p.position.y = obs.maxBounds.y; p.previous_position.y = p.position.y; }
        else if (minDist == distToMinZ) { p.position.z = obs.minBounds.z; p.previous_position.z = p.position.z; }
        else if (minDist == distToMaxZ) { p.position.z = obs.maxBounds.z; p.previous_position.z = p.position.z; }
    }
}

// ==========================================
// 4. LOADERS & THE SUSPENSION WEB
// ==========================================
void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2, float stiffness, float damping, bool isSusp) {
    float dist = Distance(p1.position, p2.position);
    springs.push_back({&p1, &p2, dist, dist, stiffness, damping, isSusp}); // Note: dist is stored twice (rest & original)
}

void LoadPhysicsCage(const char* fileName, std::vector<Particle>& particles, std::vector<Spring>& springs, float stiffness, Vector3 spawnOffset) {
    std::ifstream file(fileName);
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix; iss >> prefix;
        if (prefix == "v") {
            Vector3 pos; iss >> pos.x >> pos.y >> pos.z;
            pos = Add(pos, spawnOffset);
            particles.push_back({pos, pos, {0,0,0}, 1.0f, false}); 
        }
    }
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            CreateSpring(springs, particles[i], particles[j], stiffness, 4.0f, false);
        }
    }
}

void LoadWheelsAndSuspension(const char* fileName, std::vector<Particle>& wheels, std::vector<Particle>& cage, std::vector<Spring>& springs, Vector3 spawnOffset) {
    std::ifstream file(fileName);
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix; iss >> prefix;
        if (prefix == "v") {
            Vector3 pos; iss >> pos.x >> pos.y >> pos.z;
            pos = Add(pos, spawnOffset);
            wheels.push_back({pos, pos, {0,0,0}, 4.0f, true}); 
        }
    }
    
    float suspStiffness = 600.0f; 
    float suspDamping = 30.0f;
    for (auto& w : wheels) {
        for (size_t i = 0; i < cage.size(); i++) {
            CreateSpring(springs, w, cage[i], suspStiffness, suspDamping, true);
        }
    }
}

void LoadVisualSkin(const char* fileName, std::vector<VisualVertex>& visualVertices, std::vector<int>& indices, Vector3 spawnOffset) {
    std::ifstream file(fileName);
    if (!file.is_open()) return;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix; iss >> prefix;
        if (prefix == "v") {
            Vector3 pos; iss >> pos.x >> pos.y >> pos.z;
            pos = Add(pos, spawnOffset);
            visualVertices.push_back({pos, 0, {0,0,0}});
        } else if (prefix == "f") {
            int v[3];
            for (int i = 0; i < 3; i++) {
                std::string vData; iss >> vData;
                std::stringstream vStream(vData.substr(0, vData.find('/')));
                vStream >> v[i];
                indices.push_back(v[i] - 1);
            }
        }
    }
}

void BindSkinToCage(std::vector<VisualVertex>& visualVertices, const std::vector<Particle>& particles) {
    if (particles.empty()) return;
    float minZ = 9999, maxZ = -9999, maxY = -9999;
    for(size_t i = 0; i < particles.size(); i++) {
        if(particles[i].position.z < minZ) { minZ = particles[i].position.z; pBack = i; }
        if(particles[i].position.z > maxZ) { maxZ = particles[i].position.z; pFront = i; }
        if(particles[i].position.y > maxY) { maxY = particles[i].position.y; pTop = i; }
    }

    Vector3 cBack = particles[pBack].position;
    Vector3 cFront = particles[pFront].position;
    Vector3 cTop = particles[pTop].position;

    Vector3 forward = Normalize(Subtract(cFront, cBack));
    Vector3 tempUp = Normalize(Subtract(cTop, cBack));
    Vector3 right = Normalize(Cross(tempUp, forward));
    Vector3 up = Cross(forward, right);

    for (auto& vv : visualVertices) {
        float minDistance = 999999.0f; int closestIndex = 0;
        for (size_t i = 0; i < particles.size(); i++) {
            float dist = Distance(vv.position, particles[i].position);
            if (dist < minDistance) { minDistance = dist; closestIndex = i; }
        }
        vv.boundParticleIndex = closestIndex;
        Vector3 globalOffset = Subtract(vv.position, particles[closestIndex].position);
        vv.offset.x = Dot(globalOffset, right);
        vv.offset.y = Dot(globalOffset, up);
        vv.offset.z = Dot(globalOffset, forward);
    }
}

// ==========================================
// 5. MAIN ENGINE LOOP
// ==========================================
int main() {
    InitWindow(1280, 720, "Vortex Engine - Stable Performance Core");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 15.0f, 15.0f, 20.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    std::vector<Particle> cageParticles;
    std::vector<Particle> wheels;
    std::vector<Spring> allSprings; 
    
    std::vector<VisualVertex> carSkin;
    std::vector<int> carIndices;

    Vector3 spawnPoint = {0.0f, 3.0f, 5.0f}; 
    
    LoadPhysicsCage("cage.obj", cageParticles, allSprings, 600.0f, spawnPoint);
    LoadWheelsAndSuspension("wheels.obj", wheels, cageParticles, allSprings, spawnPoint);
    LoadVisualSkin("Car.obj", carSkin, carIndices, spawnPoint);
    BindSkinToCage(carSkin, cageParticles);

    // --- NEW: LOAD REAL WHEELS ---
    Model wheelModels[4];
    wheelModels[0] = LoadModel("wheel_fl.obj");
    wheelModels[1] = LoadModel("wheel_fr.obj");
    wheelModels[2] = LoadModel("wheel_rl.obj");
    wheelModels[3] = LoadModel("wheel_rr.obj");

    // Map Physics Wheels Front/Rear based on Z
    std::vector<int> frontWheels, rearWheels;
    int fl = 0, fr = 1, rl = 2, rr = 3; 
    if (wheels.size() == 4) {
        std::vector<std::pair<float, int>> zSort;
        for(int i=0; i<4; i++) zSort.push_back({wheels[i].position.z, i});
        std::sort(zSort.begin(), zSort.end());
        
        // Z+ is Front, Z- is Rear
        rl = zSort[0].second; rr = zSort[1].second;
        fl = zSort[2].second; fr = zSort[3].second;
        
        rearWheels.push_back(rl); rearWheels.push_back(rr);
        frontWheels.push_back(fl); frontWheels.push_back(fr);
    }

    // Larger Pillar for reliable crash testing
    Obstacle pillar = { {-3.0f, 0.0f, 20.0f}, {3.0f, 10.0f, 25.0f}, GRAY };

    float gravity = -15.0f; 
    float wheelRadius = 0.4f; 
    float maxEnginePower = 2000.0f; // Slightly more power for the heavy feel

    // HEAVY MOMENTUM VARIABLES
    float currentGas = 0.0f;
    float currentSteering = 0.0f;
    float visualWheelRot = 0.0f; // Tracks wheel spin

    float camYaw   = 0.0f;   // left/right angle around car
    float camPitch = 0.4f;   // up/down angle (radians)
    float camDist  = 12.0f;  // distance from car
    DisableCursor();          // capture & hide mouse like GTA





    Vector3 initFwd = Normalize(Subtract(cageParticles[pFront].position, cageParticles[pBack].position));


    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

         // --- GTA CAMERA INPUT ---
        Vector2 mouseDelta = GetMouseDelta();
        camYaw   -= mouseDelta.x * 0.004f;
        camPitch -= mouseDelta.y * 0.004f;
        camPitch = Clamp(camPitch, 0.05f, 1.4f);
        float scroll = GetMouseWheelMove();
        camDist -= scroll * 1.5f;
        camDist = Clamp(camDist, 4.0f, 30.0f);

        Vector3 cBack = cageParticles[pBack].position;
        Vector3 cFront = cageParticles[pFront].position;
        Vector3 cTop = cageParticles[pTop].position;
        Vector3 carForward = Normalize(Subtract(cFront, cBack));
        Vector3 tempUp = Normalize(Subtract(cTop, cBack));
        Vector3 carRight = Normalize(Cross(tempUp, carForward));
        Vector3 carUp = Cross(carForward, carRight);

        bool handbrake = IsKeyDown(KEY_SPACE);
        float targetGas = 0.0f;
        if (IsKeyDown(KEY_W)) targetGas = maxEnginePower;
        if (IsKeyDown(KEY_S)) targetGas = -maxEnginePower;
        if (handbrake) targetGas = 0.0f;  // cut engine on handbrake
        currentGas += (targetGas - currentGas) * 1.5f * dt;

        float targetSteer = 0.0f;
        if (IsKeyDown(KEY_A)) targetSteer = -0.6f;
        if (IsKeyDown(KEY_D)) targetSteer = 0.6f;
        
        currentSteering += (targetSteer - currentSteering) * 5.0f * dt;
       

        Vector3 frontWheelHeading = Add(Scale(carForward, std::cos(currentSteering)), Scale(carRight, -std::sin(currentSteering)));
        

        // --- Save wheel positions BEFORE substeps ---
        Vector3 wheelStartPos[4];
        for (int i = 0; i < 4 && i < (int)wheels.size(); i++)
            wheelStartPos[i] = wheels[i].position;

        int subSteps = 10; 
        float subDt = dt / subSteps;

        // --- PHYSICS SOLVER ---
        for (int i = 0; i < subSteps; i++) {
            
            for (auto& p : cageParticles) p.acceleration.y += gravity;
            for (auto& w : wheels) w.acceleration.y += gravity;

            for (auto& s : allSprings) ApplySpringForce(s, subDt);

            for (auto& p : cageParticles) {
                UpdateParticle(p, subDt);
                if (p.position.y < 0.2f) {
                    p.position.y = 0.2f;
                    p.previous_position.y = p.position.y + (p.position.y - p.previous_position.y) * 0.3f; 
                }
                HandleObstacleCollision(p, pillar); 
            }

            if (wheels.size() == 4) {
                for (int wIdx = 0; wIdx < 4; wIdx++) {
                    UpdateParticle(wheels[wIdx], subDt);
                    HandleObstacleCollision(wheels[wIdx], pillar); 
                    
                    if (wheels[wIdx].position.y <= wheelRadius + 0.1f) {
                        wheels[wIdx].position.y = wheelRadius;
                        wheels[wIdx].previous_position.y = wheels[wIdx].position.y; 
                        
                        Vector3 heading = (wIdx == frontWheels[0] || wIdx == frontWheels[1]) ? frontWheelHeading : carForward;
                        
                        float grip = 0.75f;
                        if (handbrake) {
                            // Front wheels: high grip (they steer and resist)
                            // Rear wheels: low grip (they slide)
                            grip = (wIdx == rearWheels[0] || wIdx == rearWheels[1]) ? 0.02f : 0.6f;
                            
                            // Kill forward momentum on ALL wheels
                            Vector3 vel = Subtract(wheels[wIdx].position, wheels[wIdx].previous_position);
                            float fwdSpeed = Dot(vel, carForward);
                            Vector3 brakeDrag = Scale(carForward, -fwdSpeed * 0.6f);
                            wheels[wIdx].acceleration = Add(wheels[wIdx].acceleration, brakeDrag);
                        } else {
                            wheels[wIdx].acceleration = Add(wheels[wIdx].acceleration, Scale(heading, currentGas));
                        }   
                        
                        ApplyTireFriction(wheels[wIdx], heading, grip);
                    }
                }
            }
        }


        // --- CORRECT WHEEL SPIN: full-frame displacement / circumference ---
        if (wheels.size() == 4) {
            // Use rear wheel for spin (unsteered, clean forward velocity)
            Vector3 rlDisplacement = Subtract(wheels[rl].position, wheelStartPos[rl]);
            float signedDist = Dot(rlDisplacement, carForward);
            visualWheelRot += signedDist / wheelRadius;
        }

        // --- VISUAL UPDATE ---
        for (auto& vv : carSkin) {
            Vector3 rotatedOffset = Add(Add(Scale(carRight, vv.offset.x), Scale(carUp, vv.offset.y)), Scale(carForward, vv.offset.z));
            vv.position = Add(cageParticles[vv.boundParticleIndex].position, rotatedOffset);
        }

        // --- GTA ORBIT CAMERA ---
        if (!cageParticles.empty()) {
            // Compute TRUE center by averaging ALL cage particles
            Vector3 carCenter = {0, 0, 0};
            for (auto& p : cageParticles) {
                carCenter.x += p.position.x;
                carCenter.y += p.position.y;
                carCenter.z += p.position.z;
            }
            float n = (float)cageParticles.size();
            carCenter.x /= n;
            carCenter.y /= n;
            carCenter.z /= n;
            carCenter.y += 1.2f; // eye-level height

            float offsetX = camDist * std::cos(camPitch) * std::sin(camYaw);
            float offsetY = camDist * std::sin(camPitch);
            float offsetZ = camDist * std::cos(camPitch) * std::cos(camYaw);

            camera.target   = carCenter;
            camera.position = { carCenter.x + offsetX,
                                carCenter.y + offsetY,
                                carCenter.z + offsetZ };
        }

        // --- PERFORMANCE RENDERER ---
        BeginDrawing();
            ClearBackground(SKYBLUE);
            BeginMode3D(camera);
                DrawGrid(40, 1.0f);
                
                DrawCube({(pillar.minBounds.x + pillar.maxBounds.x)/2, (pillar.minBounds.y + pillar.maxBounds.y)/2, (pillar.minBounds.z + pillar.maxBounds.z)/2}, 
                         pillar.maxBounds.x - pillar.minBounds.x, pillar.maxBounds.y - pillar.minBounds.y, pillar.maxBounds.z - pillar.minBounds.z, pillar.color);

                // HIGH PERFORMANCE BATCH RENDERING (Saves FPS)
                rlBegin(RL_TRIANGLES);
                for (size_t i = 0; i < carIndices.size(); i += 3) {
                    Vector3 v1 = carSkin[carIndices[i]].position;
                    Vector3 v2 = carSkin[carIndices[i+1]].position;
                    Vector3 v3 = carSkin[carIndices[i+2]].position;
                    
                    // Simple light shading
                    Vector3 n = Normalize(Cross(Subtract(v2, v1), Subtract(v3, v1)));
                    float l = std::abs(Dot(n, {0,1,0})) * 0.5f + 0.5f;
                    rlColor4ub(255*l, 0, 0, 255);
                    
                    rlVertex3f(v1.x, v1.y, v1.z); rlVertex3f(v2.x, v2.y, v2.z); rlVertex3f(v3.x, v3.y, v3.z);
                    rlVertex3f(v1.x, v1.y, v1.z); rlVertex3f(v3.x, v3.y, v3.z); rlVertex3f(v2.x, v2.y, v2.z);
                }
                rlEnd();
                
                rlBegin(RL_LINES);
                rlColor4ub(0, 0, 0, 255); // Black wireframe over top
                for (size_t i = 0; i < carIndices.size(); i += 3) {
                    Vector3 v1 = carSkin[carIndices[i]].position;
                    Vector3 v2 = carSkin[carIndices[i+1]].position;
                    Vector3 v3 = carSkin[carIndices[i+2]].position;

                    rlVertex3f(v1.x, v1.y, v1.z); rlVertex3f(v2.x, v2.y, v2.z);
                    rlVertex3f(v2.x, v2.y, v2.z); rlVertex3f(v3.x, v3.y, v3.z);
                    rlVertex3f(v3.x, v3.y, v3.z); rlVertex3f(v1.x, v1.y, v1.z);
                }
                rlEnd();

                // REAL 3D WHEEL RENDERING
                auto DrawOneWheel = [&](Model& m, int physIdx, float angle) {
                    Matrix matScale = MatrixScale(1.0f, 1.0f, 1.0f);
                    Matrix matTrans = MatrixTranslate(wheels[physIdx].position.x, wheels[physIdx].position.y, wheels[physIdx].position.z);
                    
                    // Spin (X) and Steer (Y)
                    Matrix matRot = MatrixMultiply(MatrixRotateX(visualWheelRot), MatrixRotateY(angle)); 
                    
                    // Align to chassis
                    float carAngle = atan2(carForward.x, carForward.z);
                    Matrix matBody = MatrixRotateY(carAngle);
                    
                    m.transform = MatrixMultiply(matScale, MatrixMultiply(matRot, MatrixMultiply(matBody, matTrans)));
                    DrawModel(m, {0,0,0}, 1.0f, WHITE);
                };

                // Draw actual files mapped to physics array
                DrawOneWheel(wheelModels[0], fl, -currentSteering); 
                DrawOneWheel(wheelModels[1], fr, -currentSteering); 
                DrawOneWheel(wheelModels[2], rl, 0.0f);      
                DrawOneWheel(wheelModels[3], rr, 0.0f);   

            EndMode3D();

            DrawText("V2.3 CORE: Smooth Momentum + Batch Rendering + Real Wheels", 10, 10, 20, DARKGRAY);
            DrawText("WASD: Drive | SPACE: Handbrake Drift", 10, 40, 20, BLACK);
            DrawFPS(10, 70);
        EndDrawing();
    }

    for(int i=0; i<4; i++) UnloadModel(wheelModels[i]);
    CloseWindow();
    return 0;
}