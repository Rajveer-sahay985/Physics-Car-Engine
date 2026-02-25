#include "raylib.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

// ==========================================
// 1. MATH HELPERS (Local Coordinate Systems)
// ==========================================
Vector3 Subtract(Vector3 v1, Vector3 v2) { return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z}; }
Vector3 Add(Vector3 v1, Vector3 v2) { return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z}; }
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
};

// UPGRADED: Now binds to 3 points to preserve rotation!
struct VisualVertex {
    Vector3 position;
    int pA, pB, pC; // The indices of the 3 closest physics points
    Vector3 offset; // Local XYZ offsets relative to the 3 points
};

struct Obstacle {
    Vector3 minBounds;
    Vector3 maxBounds;
    Color color;
};

// ==========================================
// 3. PHYSICS BACKEND
// ==========================================
void UpdateParticle(Particle& p, float deltaTime) {
    Vector3 velocity = Subtract(p.position, p.previous_position);
    
    // Friction
    float friction = (p.position.y <= 0.25f) ? 0.90f : 0.99f;
    velocity.x *= friction; velocity.y *= friction; velocity.z *= friction;

    p.previous_position = p.position;
    p.position.x += velocity.x + p.acceleration.x * deltaTime * deltaTime;
    p.position.y += velocity.y + p.acceleration.y * deltaTime * deltaTime;
    p.position.z += velocity.z + p.acceleration.z * deltaTime * deltaTime;
    p.acceleration = {0.0f, 0.0f, 0.0f};
}

void ApplySpringForce(Spring& spring) {
    float current_distance = Distance(spring.p1->position, spring.p2->position);
    if (current_distance == 0.0f) return;

    float displacement = current_distance - spring.rest_length;

    // --- PLASTIC DEFORMATION (CRUMPLE MATH) ---
    float yield_point = 0.3f; // Force required to permanently bend the metal
    if (std::abs(displacement) > yield_point) {
        // Permanently alter the rest length. 0.05f is the plasticity rate.
        spring.rest_length += displacement * 0.05f; 
    }

    float force = spring.stiffness * displacement;
    Vector3 dir = Normalize(Subtract(spring.p2->position, spring.p1->position));

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
// 4. PARSERS & BINDING
// ==========================================
void CreateSpring(std::vector<Spring>& springs, Particle& p1, Particle& p2, float stiffness) {
    springs.push_back({&p1, &p2, Distance(p1.position, p2.position), stiffness});
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
            visualVertices.push_back({pos, 0, 0, 0, {0,0,0}});
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

// UPGRADED BINDING: Finds 3 points and creates a local coordinate compass
void BindSkinToCage(std::vector<VisualVertex>& visualVertices, const std::vector<Particle>& particles) {
    if (particles.size() < 3) return;

    for (auto& vv : visualVertices) {
        // Find the 3 closest cage particles
        std::vector<std::pair<float, int>> dists;
        for (size_t i = 0; i < particles.size(); i++) {
            dists.push_back({Distance(vv.position, particles[i].position), i});
        }
        std::sort(dists.begin(), dists.end());
        
        vv.pA = dists[0].second;
        vv.pB = dists[1].second;
        vv.pC = dists[2].second;

        Vector3 pA = particles[vv.pA].position;
        Vector3 pB = particles[vv.pB].position;
        Vector3 pC = particles[vv.pC].position;

        // Build a local XYZ compass for this specific vertex
        Vector3 x_axis = Normalize(Subtract(pB, pA));
        Vector3 temp = Normalize(Subtract(pC, pA));
        Vector3 y_axis = Normalize(Cross(x_axis, temp));
        Vector3 z_axis = Cross(x_axis, y_axis);

        // Store its offset relative to this compass, not the world
        Vector3 diff = Subtract(vv.position, pA);
        vv.offset.x = Dot(diff, x_axis);
        vv.offset.y = Dot(diff, y_axis);
        vv.offset.z = Dot(diff, z_axis);
    }
}

// ==========================================
// 5. MAIN ENGINE LOOP
// ==========================================
int main() {
    InitWindow(1024, 768, "Vortex Engine V1.1 - Plastic Deformation");
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
    
    LoadPhysicsCage("cage.obj", cageParticles, cageSprings, 300.0f, spawnPoint);
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

            for (auto& s : cageSprings) ApplySpringForce(s);

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

        // UPGRADED VISUAL UPDATE: Reconstruct position using the local compass
        for (auto& vv : carSkin) {
            Vector3 pA = cageParticles[vv.pA].position;
            Vector3 pB = cageParticles[vv.pB].position;
            Vector3 pC = cageParticles[vv.pC].position;

            Vector3 x_axis = Normalize(Subtract(pB, pA));
            Vector3 temp = Normalize(Subtract(pC, pA));
            Vector3 y_axis = Normalize(Cross(x_axis, temp));
            Vector3 z_axis = Cross(x_axis, y_axis);

            vv.position.x = pA.x + x_axis.x * vv.offset.x + y_axis.x * vv.offset.y + z_axis.x * vv.offset.z;
            vv.position.y = pA.y + x_axis.y * vv.offset.x + y_axis.y * vv.offset.y + z_axis.y * vv.offset.z;
            vv.position.z = pA.z + x_axis.z * vv.offset.x + y_axis.z * vv.offset.y + z_axis.z * vv.offset.z;
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

                for (size_t i = 0; i < carIndices.size(); i += 3) {
                    Vector3 v1 = carSkin[carIndices[i]].position;
                    Vector3 v2 = carSkin[carIndices[i+1]].position;
                    Vector3 v3 = carSkin[carIndices[i+2]].position;
                    DrawTriangle3D(v1, v2, v3, RED);
                    DrawTriangle3D(v3, v2, v1, RED); 
                }
                // The cage rendering loop is completely gone!
            EndMode3D();

            DrawText("Plastic Crumple Active. WASD to Drive", 10, 10, 20, DARKGRAY);
            DrawFPS(10, 40);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}