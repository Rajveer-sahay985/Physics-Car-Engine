#include "raylib.h"
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
// 3. PHYSICS BACKEND (The Golden Era Logic)
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

    if (!spring.isSuspension) {
        float yield_point = 0.4f; 
        if (std::abs(displacement) > yield_point) {
            spring.rest_length += displacement * 0.05f; 
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

        if (minDist == distToMinX) p.position.x = obs.minBounds.x;
        else if (minDist == distToMaxX) p.position.x = obs.maxBounds.x;
        else if (minDist == distToMinY) p.position.y = obs.minBounds.y;
        else if (minDist == distToMaxY) p.position.y = obs.maxBounds.y;
        else if (minDist == distToMinZ) p.position.z = obs.minBounds.z;
        else if (minDist == distToMaxZ) p.position.z = obs.maxBounds.z;
        
        p.previous_position = p.position;
    }
}

// ==========================================
// 4. LOADERS
// ==========================================
void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2, float stiffness, float damping, bool isSusp) {
    springs.push_back({&p1, &p2, Distance(p1.position, p2.position), stiffness, damping, isSusp});
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
    InitWindow(1024, 768, "Vortex Engine V2.4 - Fixed Steering Visuals");
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

    // Identify Front/Rear
    std::vector<int> frontWheels, rearWheels;
    if (wheels.size() == 4) {
        std::vector<std::pair<float, int>> zSort;
        for(int i=0; i<4; i++) zSort.push_back({wheels[i].position.z, i});
        std::sort(zSort.begin(), zSort.end());
        rearWheels.push_back(zSort[0].second); rearWheels.push_back(zSort[1].second);
        frontWheels.push_back(zSort[2].second); frontWheels.push_back(zSort[3].second);
    }

    Obstacle pillar = { {-2.0f, 0.0f, -15.0f}, {2.0f, 10.0f, -11.0f}, GRAY };

    float gravity = -25.0f; 
    float wheelRadius = 0.4f; 
    float maxEnginePower = 1500.0f;

    // INPUT SMOOTHING VARIABLES (Inertia)
    float currentGas = 0.0f;
    float currentSteering = 0.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        Vector3 cBack = cageParticles[pBack].position;
        Vector3 cFront = cageParticles[pFront].position;
        Vector3 cTop = cageParticles[pTop].position;
        Vector3 carForward = Normalize(Subtract(cFront, cBack));
        Vector3 tempUp = Normalize(Subtract(cTop, cBack));
        Vector3 carRight = Normalize(Cross(tempUp, carForward));
        Vector3 carUp = Cross(carForward, carRight);

        // --- SMOOTHED INPUTS ---
        float targetGas = 0.0f;
        if (IsKeyDown(KEY_W)) targetGas = maxEnginePower;
        if (IsKeyDown(KEY_S)) targetGas = -maxEnginePower; 
        
        // Lerp gas for weight feel (0.1f speed)
        currentGas += (targetGas - currentGas) * 5.0f * dt;

        float targetSteer = 0.0f;
        if (IsKeyDown(KEY_A)) targetSteer = 0.6f;  // Turn Left
        else if (IsKeyDown(KEY_D)) targetSteer = -0.6f; // Turn Right
        
        // Lerp steering for handling feel
        currentSteering += (targetSteer - currentSteering) * 8.0f * dt;

        // Physics uses currentSteering
        Vector3 frontWheelHeading = Add(Scale(carForward, std::cos(currentSteering)), Scale(carRight, std::sin(currentSteering)));

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
                        
                        // Use smoothed gas
                        wheels[wIdx].acceleration = Add(wheels[wIdx].acceleration, Scale(heading, currentGas));
                        
                        ApplyTireFriction(wheels[wIdx], heading, 0.08f); 
                    }
                }
            }
        }

        for (auto& vv : carSkin) {
            Vector3 rotatedOffset = Add(Add(Scale(carRight, vv.offset.x), Scale(carUp, vv.offset.y)), Scale(carForward, vv.offset.z));
            vv.position = Add(cageParticles[vv.boundParticleIndex].position, rotatedOffset);
        }
        
        if (!cageParticles.empty()) {
            camera.target = cageParticles[0].position;
            camera.position = { cageParticles[0].position.x + 10.0f, 10.0f, cageParticles[0].position.z + 15.0f };
        }

        // --- RENDER ---
        BeginDrawing();
            ClearBackground(SKYBLUE);
            BeginMode3D(camera);
                DrawGrid(40, 1.0f);
                
                DrawCube({(pillar.minBounds.x + pillar.maxBounds.x)/2, (pillar.minBounds.y + pillar.maxBounds.y)/2, (pillar.minBounds.z + pillar.maxBounds.z)/2}, 
                         pillar.maxBounds.x - pillar.minBounds.x, pillar.maxBounds.y - pillar.minBounds.y, pillar.maxBounds.z - pillar.minBounds.z, pillar.color);

                for (size_t i = 0; i < carIndices.size(); i += 3) {
                    Vector3 v1 = carSkin[carIndices[i]].position;
                    Vector3 v2 = carSkin[carIndices[i+1]].position;
                    Vector3 v3 = carSkin[carIndices[i+2]].position;
                    DrawLine3D(v1, v2, RED);
                    DrawLine3D(v2, v3, RED);
                    DrawLine3D(v3, v1, RED);
                }

                // DRAW VISUAL WHEELS (The Fix Is Here)
                for(size_t wIdx = 0; wIdx < wheels.size(); wIdx++) {
                    Vector3 axleDir = carRight;
                    if (wheels.size() == 4 && (wIdx == frontWheels[0] || wIdx == frontWheels[1])) {
                        // VISUAL FIX: Use NEGATIVE steering for the visual rotation axis
                        // This flips the visual rotation to match the physical turn
                        axleDir = Add(Scale(carRight, std::cos(currentSteering)), Scale(carForward, std::sin(-currentSteering)));
                    }
                    
                    Vector3 offset = Scale(axleDir, 0.2f);
                    Vector3 wheelStart = Subtract(wheels[wIdx].position, offset);
                    Vector3 wheelEnd = Add(wheels[wIdx].position, offset);
                    
                    DrawCylinderEx(wheelStart, wheelEnd, wheelRadius, wheelRadius, 16, DARKGRAY);
                }

            EndMode3D();

            DrawText("Visual Steering Fixed + Smooth Controls.", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}