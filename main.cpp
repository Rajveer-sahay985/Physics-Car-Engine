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
};

struct Spring {
    Particle* p1;
    Particle* p2;
    float rest_length;
    float stiffness;
    float damping; 
};

struct VisualVertex {
    Vector3 position;
    int boundParticleIndex; // The closest node for CRUMPLING
    Vector3 offset;         // Master Compass offset for ROTATION
};

struct Obstacle {
    Vector3 minBounds;
    Vector3 maxBounds;
    Color color;
};

// Global indices for our Master Compass
int pFront = 0, pBack = 0, pTop = 0; 

// ==========================================
// 3. PHYSICS BACKEND
// ==========================================
void UpdateParticle(Particle& p, float deltaTime) {
    Vector3 velocity = Subtract(p.position, p.previous_position);
    
    float friction = (p.position.y <= 0.25f) ? 0.90f : 0.99f;
    velocity.x *= friction; velocity.y *= friction; velocity.z *= friction;

    p.previous_position = p.position;
    p.position.x += velocity.x + p.acceleration.x * deltaTime * deltaTime;
    p.position.y += velocity.y + p.acceleration.y * deltaTime * deltaTime;
    p.position.z += velocity.z + p.acceleration.z * deltaTime * deltaTime;
    p.acceleration = {0.0f, 0.0f, 0.0f};
}

void ApplySpringForce(Spring& spring, float deltaTime) {
    float current_distance = Distance(spring.p1->position, spring.p2->position);
    if (current_distance == 0.0f) return;

    Vector3 dir = Normalize(Subtract(spring.p2->position, spring.p1->position));
    float displacement = current_distance - spring.rest_length;

    // Plastic Deformation (Crumpling)
    float yield_point = 0.4f; 
    if (std::abs(displacement) > yield_point) {
        spring.rest_length += displacement * 0.05f; 
    }

    // Damping Math
    Vector3 v1 = Subtract(spring.p1->position, spring.p1->previous_position);
    Vector3 v2 = Subtract(spring.p2->position, spring.p2->previous_position);
    Vector3 relVel = Subtract(v2, v1);
    float dampForce = Dot(relVel, dir) * spring.damping;

    float force = (spring.stiffness * displacement) + dampForce;

    float acc1 = force / spring.p1->mass;
    float acc2 = force / spring.p2->mass;

    spring.p1->acceleration.x += dir.x * acc1;
    spring.p1->acceleration.y += dir.y * acc1;
    spring.p1->acceleration.z += dir.z * acc1;

    spring.p2->acceleration.x -= dir.x * acc2;
    spring.p2->acceleration.y -= dir.y * acc2;
    spring.p2->acceleration.z -= dir.z * acc2;
}

void HandleObstacleCollision(Particle& p, const Obstacle& obs) {
    if (p.position.x > obs.minBounds.x && p.position.x < obs.maxBounds.x &&
        p.position.y > obs.minBounds.y && p.position.y < obs.maxBounds.y &&
        p.position.z > obs.minBounds.z && p.position.z < obs.maxBounds.z) {
        
        float distToMinX = p.position.x - obs.minBounds.x;
        float distToMaxX = obs.maxBounds.x - p.position.x;
        float distToMinY = p.position.y - obs.minBounds.y;
        float distToMaxY = obs.maxBounds.y - p.position.y;
        float distToMinZ = p.position.z - obs.minBounds.z;
        float distToMaxZ = obs.maxBounds.z - p.position.z;

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
// 4. PARSERS & MASTER BINDING
// ==========================================
void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2, float stiffness) {
    springs.push_back({&p1, &p2, Distance(p1.position, p2.position), stiffness, 3.0f});
}

void LoadPhysicsCage(const char* fileName, std::vector<Particle>& particles, std::vector<Spring>& springs, float stiffness, Vector3 spawnOffset) {
    std::ifstream file(fileName);
    if (!file.is_open()) return;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if (prefix == "v") {
            Vector3 pos;
            iss >> pos.x >> pos.y >> pos.z;
            pos = Add(pos, spawnOffset);
            particles.push_back({pos, pos, {0.0f, 0.0f, 0.0f}, 1.0f});
        }
    }
    file.close();

    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            CreateSpring(springs, particles[i], particles[j], stiffness);
        }
    }
}

void LoadVisualSkin(const char* fileName, std::vector<VisualVertex>& visualVertices, std::vector<int>& indices, Vector3 spawnOffset) {
    std::ifstream file(fileName);
    if (!file.is_open()) return;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if (prefix == "v") {
            Vector3 pos;
            iss >> pos.x >> pos.y >> pos.z;
            pos = Add(pos, spawnOffset);
            visualVertices.push_back({pos, 0, {0,0,0}});
        } 
        else if (prefix == "f") {
            int v[3];
            for (int i = 0; i < 3; i++) {
                std::string vData;
                iss >> vData;
                std::stringstream vStream(vData.substr(0, vData.find('/')));
                vStream >> v[i];
                indices.push_back(v[i] - 1);
            }
        }
    }
}

void BindSkinToCage(std::vector<VisualVertex>& visualVertices, const std::vector<Particle>& particles) {
    if (particles.empty()) return;

    // 1. Find the absolute extremities of the cage to create the Master Compass
    float minZ = 9999, maxZ = -9999, maxY = -9999;
    for(size_t i = 0; i < particles.size(); i++) {
        if(particles[i].position.z < minZ) { minZ = particles[i].position.z; pBack = i; }
        if(particles[i].position.z > maxZ) { maxZ = particles[i].position.z; pFront = i; }
        if(particles[i].position.y > maxY) { maxY = particles[i].position.y; pTop = i; }
    }

    // 2. Build the initial Master Compass
    Vector3 cBack = particles[pBack].position;
    Vector3 cFront = particles[pFront].position;
    Vector3 cTop = particles[pTop].position;

    Vector3 forward = Normalize(Subtract(cFront, cBack));
    Vector3 tempUp = Normalize(Subtract(cTop, cBack));
    Vector3 right = Normalize(Cross(tempUp, forward));
    Vector3 up = Cross(forward, right);

    for (auto& vv : visualVertices) {
        // Find the single closest cage node for local crumpling
        float minDistance = 999999.0f;
        int closestIndex = 0;
        for (size_t i = 0; i < particles.size(); i++) {
            float dist = Distance(vv.position, particles[i].position);
            if (dist < minDistance) { minDistance = dist; closestIndex = i; }
        }
        vv.boundParticleIndex = closestIndex;

        // Save the offset relative to the Master Compass
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
    InitWindow(1024, 768, "Vortex Engine V1.3 - Stable Wireframe Edition");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 15.0f, 15.0f, 20.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    std::vector<Particle> cageParticles;
    std::vector<Spring> cageSprings;
    std::vector<VisualVertex> carSkin;
    std::vector<int> carIndices;

    Vector3 spawnPoint = {0.0f, 5.0f, 0.0f};
    
    LoadPhysicsCage("cage.obj", cageParticles, cageSprings, 400.0f, spawnPoint);
    LoadVisualSkin("Car.obj", carSkin, carIndices, spawnPoint);
    BindSkinToCage(carSkin, cageParticles);

    Obstacle pillar = { {-5.0f, 0.0f, -5.0f}, {-2.0f, 10.0f, -2.0f}, GRAY };

    float gravity = -15.0f; 
    float driveSpeed = 80.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        Vector3 driveForce = {0.0f, 0.0f, 0.0f};
        if (IsKeyDown(KEY_W)) driveForce.z -= driveSpeed;
        if (IsKeyDown(KEY_S)) driveForce.z += driveSpeed;
        if (IsKeyDown(KEY_A)) driveForce.x -= driveSpeed;
        if (IsKeyDown(KEY_D)) driveForce.x += driveSpeed;

        int subSteps = 10; 
        float subDt = dt / subSteps;

        for (int i = 0; i < subSteps; i++) {
            for (auto& p : cageParticles) {
                p.acceleration.y += gravity;
                p.acceleration.x += driveForce.x;
                p.acceleration.z += driveForce.z;
            }

            for (auto& s : cageSprings) ApplySpringForce(s, subDt);

            for (auto& p : cageParticles) {
                UpdateParticle(p, subDt);
                if (p.position.y < 0.2f) {
                    p.position.y = 0.2f;
                    float vy = p.position.y - p.previous_position.y;
                    p.previous_position.y = p.position.y + (vy * 0.3f); 
                }
                HandleObstacleCollision(p, pillar);
            }
        }

        // RECALCULATE MASTER COMPASS dynamically every frame
        Vector3 cBack = cageParticles[pBack].position;
        Vector3 cFront = cageParticles[pFront].position;
        Vector3 cTop = cageParticles[pTop].position;

        Vector3 forward = Normalize(Subtract(cFront, cBack));
        Vector3 tempUp = Normalize(Subtract(cTop, cBack));
        Vector3 right = Normalize(Cross(tempUp, forward));
        Vector3 up = Cross(forward, right);

        // Update the visual car skin perfectly smoothly
        for (auto& vv : carSkin) {
            Vector3 rotatedOffset = Add(Add(Scale(right, vv.offset.x), Scale(up, vv.offset.y)), Scale(forward, vv.offset.z));
            vv.position = Add(cageParticles[vv.boundParticleIndex].position, rotatedOffset);
        }

        if (!cageParticles.empty()) {
            camera.target = cageParticles[0].position;
            camera.position = { cageParticles[0].position.x + 10.0f, 10.0f, cageParticles[0].position.z + 15.0f };
        }

        BeginDrawing();
            ClearBackground(SKYBLUE);
            BeginMode3D(camera);
                DrawGrid(40, 1.0f);
                
                DrawCube({(pillar.minBounds.x + pillar.maxBounds.x)/2, (pillar.minBounds.y + pillar.maxBounds.y)/2, (pillar.minBounds.z + pillar.maxBounds.z)/2}, 
                         pillar.maxBounds.x - pillar.minBounds.x, pillar.maxBounds.y - pillar.minBounds.y, pillar.maxBounds.z - pillar.minBounds.z, pillar.color);

                // PURE WIREFRAME RENDER: Max performance, no heavy shading
                for (size_t i = 0; i < carIndices.size(); i += 3) {
                    Vector3 v1 = carSkin[carIndices[i]].position;
                    Vector3 v2 = carSkin[carIndices[i+1]].position;
                    Vector3 v3 = carSkin[carIndices[i+2]].position;
                    
                    DrawLine3D(v1, v2, RED);
                    DrawLine3D(v2, v3, RED);
                    DrawLine3D(v3, v1, RED);
                }
            EndMode3D();

            DrawText("Master Compass Wireframe. Flips beautifully!", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}