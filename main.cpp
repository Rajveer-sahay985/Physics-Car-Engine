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

// Rotates a vector around the Y axis (Used for steering the front wheels)
Vector3 RotateY(Vector3 v, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return { v.x * c + v.z * s, v.y, -v.x * s + v.z * c };
}

// ==========================================
// 2. DATA STRUCTURES
// ==========================================
struct Particle {
    Vector3 position;
    Vector3 previous_position;
    Vector3 acceleration;
    float mass;
    bool isWheel; // Flags if this particle is a tire
};

struct Spring {
    Particle* p1;
    Particle* p2;
    float rest_length;
    float stiffness;
    float damping; 
    bool isSuspension; // Suspension springs don't permanently crumple
};

struct VisualVertex {
    Vector3 position;
    int boundParticleIndex; 
    Vector3 offset;         
};

int pFront = 0, pBack = 0, pTop = 0; 

// ==========================================
// 3. PHYSICS BACKEND
// ==========================================
void UpdateParticle(Particle& p, float deltaTime) {
    Vector3 velocity = Subtract(p.position, p.previous_position);
    
    // Air resistance/baseline friction
    float friction = 0.99f; 
    velocity = Scale(velocity, friction);

    p.previous_position = p.position;
    p.position.x += velocity.x + p.acceleration.x * deltaTime * deltaTime;
    p.position.y += velocity.y + p.acceleration.y * deltaTime * deltaTime;
    p.position.z += velocity.z + p.acceleration.z * deltaTime * deltaTime;
    p.acceleration = {0.0f, 0.0f, 0.0f};
}

// Kills sideways velocity so the tires grip the road instead of sliding like ice
void ApplyTireFriction(Particle& wheel, Vector3 heading, float grip) {
    Vector3 vel = Subtract(wheel.position, wheel.previous_position);
    
    float forwardSpeed = Dot(vel, heading);
    Vector3 forwardVel = Scale(heading, forwardSpeed);
    Vector3 lateralVel = Subtract(vel, forwardVel);
    
    // Reduce lateral velocity based on grip (0.0 = ice, 1.0 = rails)
    lateralVel = Scale(lateralVel, 1.0f - grip);
    
    // Reconstruct the new velocity and update the previous position to enforce it
    Vector3 newVel = Add(forwardVel, lateralVel);
    wheel.previous_position = Subtract(wheel.position, newVel);
}

void ApplySpringForce(Spring& spring, float deltaTime) {
    float current_distance = Distance(spring.p1->position, spring.p2->position);
    if (current_distance == 0.0f) return;

    Vector3 dir = Normalize(Subtract(spring.p2->position, spring.p1->position));
    float displacement = current_distance - spring.rest_length;

    // Plastic Deformation (Only apply to the chassis cage, NOT the suspension)
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

// ==========================================
// 4. LOADERS & BINDING
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
            // Chassis particles have standard mass
            particles.push_back({pos, pos, {0,0,0}, 1.0f, false}); 
        }
    }
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            CreateSpring(springs, particles[i], particles[j], stiffness, 3.0f, false);
        }
    }
}

// Loads the 4 wheel vertices and builds the suspension
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
            // Wheels are heavy (mass 3.0) to keep the car grounded
            wheels.push_back({pos, pos, {0,0,0}, 3.0f, true}); 
        }
    }
    
    // Build Suspension: Connect each wheel to the 4 nearest cage points
    float suspStiffness = 800.0f;
    float suspDamping = 15.0f;
    for (auto& w : wheels) {
        std::vector<std::pair<float, int>> dists;
        for (size_t i = 0; i < cage.size(); i++) {
            dists.push_back({Distance(w.position, cage[i].position), i});
        }
        std::sort(dists.begin(), dists.end());
        for(int i = 0; i < 4; i++) {
            CreateSpring(springs, w, cage[dists[i].second], suspStiffness, suspDamping, true);
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
    InitWindow(1024, 768, "Vortex Engine V2.0 - Drivetrain & Suspension");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 15.0f, 15.0f, 20.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    std::vector<Particle> cageParticles;
    std::vector<Particle> wheels;
    std::vector<Spring> allSprings; // Holds both cage structure AND suspension
    
    std::vector<VisualVertex> carSkin;
    std::vector<int> carIndices;

    // Spawn point slightly higher so the suspension can settle
    Vector3 spawnPoint = {0.0f, 3.0f, 0.0f}; 
    
    LoadPhysicsCage("cage.obj", cageParticles, allSprings, 400.0f, spawnPoint);
    LoadWheelsAndSuspension("wheels.obj", wheels, cageParticles, allSprings, spawnPoint);
    
    LoadVisualSkin("Car.obj", carSkin, carIndices, spawnPoint);
    BindSkinToCage(carSkin, cageParticles);

    // Identify Front and Rear Wheels based on Z position
    std::vector<int> frontWheels, rearWheels;
    if (wheels.size() == 4) {
        std::vector<std::pair<float, int>> zSort;
        for(int i=0; i<4; i++) zSort.push_back({wheels[i].position.z, i});
        std::sort(zSort.begin(), zSort.end());
        // Lowest Z are rear, Highest Z are front (assuming Z+ is forward)
        rearWheels.push_back(zSort[0].second); rearWheels.push_back(zSort[1].second);
        frontWheels.push_back(zSort[2].second); frontWheels.push_back(zSort[3].second);
    }

    float gravity = -15.0f; 
    float wheelRadius = 0.4f; // Radius of the mathematical forcefield

    float enginePower = 300.0f;
    float steeringAngle = 0.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        // Calculate Master Compass to know which way the car is pointing
        Vector3 cBack = cageParticles[pBack].position;
        Vector3 cFront = cageParticles[pFront].position;
        Vector3 cTop = cageParticles[pTop].position;
        Vector3 carForward = Normalize(Subtract(cFront, cBack));
        Vector3 tempUp = Normalize(Subtract(cTop, cBack));
        Vector3 carRight = Normalize(Cross(tempUp, carForward));
        Vector3 carUp = Cross(carForward, carRight);

        // --- INPUT: DRIVETRAIN & STEERING ---
        float gas = 0.0f;
        if (IsKeyDown(KEY_W)) gas = enginePower;
        if (IsKeyDown(KEY_S)) gas = -enginePower; // Reverse/Brake

        // Steering logic (Max 35 degree turn angle)
        if (IsKeyDown(KEY_A)) steeringAngle += 2.0f * dt;
        else if (IsKeyDown(KEY_D)) steeringAngle -= 2.0f * dt;
        else steeringAngle *= 0.8f; // Auto-center steering
        if (steeringAngle > 0.6f) steeringAngle = 0.6f;
        if (steeringAngle < -0.6f) steeringAngle = -0.6f;

        // Calculate the direction the front tires are pointing
        Vector3 frontWheelHeading = RotateY(carForward, steeringAngle);

        int subSteps = 10; 
        float subDt = dt / subSteps;

        // --- PHYSICS SOLVER ---
        for (int i = 0; i < subSteps; i++) {
            
            // 1. Apply Gravity to everything
            for (auto& p : cageParticles) p.acceleration.y += gravity;
            for (auto& w : wheels) w.acceleration.y += gravity;

            // 2. Apply Engine Force (All-Wheel Drive)
            if (wheels.size() == 4) {
                // Front wheels push along their steered heading
                wheels[frontWheels[0]].acceleration = Add(wheels[frontWheels[0]].acceleration, Scale(frontWheelHeading, gas));
                wheels[frontWheels[1]].acceleration = Add(wheels[frontWheels[1]].acceleration, Scale(frontWheelHeading, gas));
                // Rear wheels push straight ahead
                wheels[rearWheels[0]].acceleration = Add(wheels[rearWheels[0]].acceleration, Scale(carForward, gas));
                wheels[rearWheels[1]].acceleration = Add(wheels[rearWheels[1]].acceleration, Scale(carForward, gas));
            }

            // 3. Apply Springs (Cage Structure + Suspension)
            for (auto& s : allSprings) ApplySpringForce(s, subDt);

            // 4. Update Chassis Particles
            for (auto& p : cageParticles) {
                UpdateParticle(p, subDt);
                // The cage should almost never hit the floor unless you crash hard!
                if (p.position.y < 0.2f) {
                    p.position.y = 0.2f;
                    p.previous_position.y = p.position.y + (p.position.y - p.previous_position.y) * 0.3f; 
                }
            }

            // 5. Update Wheel Particles (The Magic Sauce)
            if (wheels.size() == 4) {
                for (int wIdx = 0; wIdx < 4; wIdx++) {
                    UpdateParticle(wheels[wIdx], subDt);
                    
                    // Wheel Floor Collision (Floating on the radius bubble)
                    if (wheels[wIdx].position.y < wheelRadius) {
                        wheels[wIdx].position.y = wheelRadius;
                        wheels[wIdx].previous_position.y = wheels[wIdx].position.y; 
                        
                        // If touching the ground, apply tire friction to steer!
                        Vector3 heading = (wIdx == frontWheels[0] || wIdx == frontWheels[1]) ? frontWheelHeading : carForward;
                        ApplyTireFriction(wheels[wIdx], heading, 0.85f); // 0.85 grip prevents sliding
                    }
                }
            }
        }

        // --- VISUAL UPDATE ---
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
                
                // Draw Wireframe Car Skin
                for (size_t i = 0; i < carIndices.size(); i += 3) {
                    Vector3 v1 = carSkin[carIndices[i]].position;
                    Vector3 v2 = carSkin[carIndices[i+1]].position;
                    Vector3 v3 = carSkin[carIndices[i+2]].position;
                    DrawLine3D(v1, v2, RED);
                    DrawLine3D(v2, v3, RED);
                    DrawLine3D(v3, v1, RED);
                }

                // Draw solid cylinders to visually represent the invisible physics wheels
                for(auto& w : wheels) {
                    DrawCylinder(w.position, wheelRadius, wheelRadius, 0.2f, 12, DARKGRAY);
                }

            EndMode3D();

            DrawText("Drivetrain Active! W/S to accelerate. A/D to steer.", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}